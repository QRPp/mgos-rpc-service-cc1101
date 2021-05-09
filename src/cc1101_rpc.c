#include <math.h>

#include <mgos-helpers/log.h>
#include <mgos-helpers/rpc.h>

#include <mgos-cc1101.h>

static struct tx_stats {
  bool busy;
  enum mgos_cc1101_tx_err err;
  struct mgos_cc1101_tx_stats st;
} txop;

static void txop_save(void *opaque) {
  struct mgos_cc1101_tx_op *op = opaque;
  txop.busy = false;
  txop.err = op->err;
  txop.st = op->st;
  free(op);
}

static void cc1101_identify_handler(struct mg_rpc_request_info *ri,
                                    void *cb_arg, struct mg_rpc_frame_info *fi,
                                    struct mg_str args) {
  struct mgos_cc1101 *cc1101 = mgos_cc1101_get_global_locked();
  struct CC1101_PARTNUM pn;
  if (!mgos_cc1101_read_reg(cc1101, CC1101_PARTNUM, &pn.val))
    mg_rpc_errorf_gt(500, "error getting CC1101 %s", "part number");
  struct CC1101_VERSION ver;
  if (!mgos_cc1101_read_reg(cc1101, CC1101_VERSION, &ver.val))
    mg_rpc_errorf_gt(500, "error getting CC1101 %s", "version");
  mg_rpc_send_responsef(ri, "{partnum:%u,version:%u}", pn.PARTNUM, ver.VERSION);
err:
  mgos_cc1101_put_global_locked();
}

#define READ_REG_FMT "{reg:%u}"
static void cc1101_read_reg_handler(struct mg_rpc_request_info *ri,
                                    void *cb_arg, struct mg_rpc_frame_info *fi,
                                    struct mg_str args) {
  unsigned reg;
  if (json_scanf(args.p, args.len, ri->args_fmt, &reg) != 1)
    mg_rpc_errorf_ret(400, "reg is required");
  uint8_t val;
  bool ok = mgos_cc1101_read_reg(mgos_cc1101_get_global_locked(), reg, &val);
  mgos_cc1101_put_global_locked();
  if (!ok) mg_rpc_errorf_ret(500, "error reading register %u", reg);
  mg_rpc_send_responsef(ri, "{reg:%u,val:%u}", reg, val);
}

#define READ_REGS_FMT "{reg:%u,cnt:%u}"
static void cc1101_read_regs_handler(struct mg_rpc_request_info *ri,
                                     void *cb_arg, struct mg_rpc_frame_info *fi,
                                     struct mg_str args) {
  unsigned reg, cnt;
  if (json_scanf(args.p, args.len, ri->args_fmt, &reg, &cnt) != 2)
    mg_rpc_errorf_ret(400, "reg and cnt are required");
  uint8_t vals[1 + cnt];
  vals[0] = reg;
  bool ok = mgos_cc1101_read_regs(mgos_cc1101_get_global_locked(), cnt, vals);
  mgos_cc1101_put_global_locked();
  if (!ok)
    mg_rpc_errorf_ret(500, "error reading %u registers from %u", cnt, reg);
  mg_rpc_send_responsef(ri, "{status:%u,reg:%u,cnt:%u,vals:%H}", vals[0], reg,
                        cnt, cnt, vals + 1);
}

static void cc1101_reset_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                                 struct mg_rpc_frame_info *fi,
                                 struct mg_str args) {
  bool ok = mgos_cc1101_reset(mgos_cc1101_get_global_locked());
  mgos_cc1101_put_global_locked();
  if (!ok) mg_rpc_errorf_ret(500, "error resetting CC1101 RF module");
  cc1101_identify_handler(ri, cb_arg, fi, args);
}

struct cc1101_set_conf_cb {
  struct mg_rpc_request_info *ri;
  float kbaud, mhz, dev_khz;
  unsigned mod_format, manchester, fec, sync_mode, white_data, pkt_format, crc,
      len_cfg;
};

static bool cc1101_set_conf_cb(struct mgos_cc1101 *cc1101,
                               struct mgos_cc1101_regs *regs, void *opaque) {
  struct cc1101_set_conf_cb *d = opaque;
#define ri d->ri

  /* Data rate.*/
  if (!isnan(d->kbaud) && !mgos_cc1101_set_data_rate(cc1101, regs, d->kbaud))
    mg_rpc_errorf_retv(false, 500, "can't set %f kbaud data rate", d->kbaud);

  /* Base frequency.*/
  if (!isnan(d->mhz) && !mgos_cc1101_set_frequency(cc1101, regs, d->mhz))
    mg_rpc_errorf_retv(false, 500, "can't set %f MHz frequency", d->mhz);

  /* Deviation frequency.*/
  if (!isnan(d->dev_khz) &&
      !mgos_cc1101_set_deviation(cc1101, regs, d->dev_khz))
    mg_rpc_errorf_retv(false, 500, "can't set %f kHz frequency deviation",
                       d->dev_khz);

  /* Modulation. */
  if (d->mod_format != UINT_MAX || d->manchester != BOOL_INVAL ||
      d->fec != BOOL_INVAL || d->sync_mode != UINT_MAX) {
    if (d->mod_format == UINT_MAX) d->mod_format = 0;
    if (d->manchester == BOOL_INVAL) d->manchester = false;
    if (d->fec == BOOL_INVAL) d->fec = false;
    if (d->sync_mode == UINT_MAX) d->sync_mode = 2;
  }
  if (d->mod_format != UINT_MAX &&
      !mgos_cc1101_set_modulation(cc1101, regs, d->mod_format, d->sync_mode,
                                  d->manchester, d->fec))
    mg_rpc_errorf_retv(false, 500,
                       "can't set modulation format %u "
                       "(Manchester %s, FEC %s, sync mode %u)",
                       d->mod_format, ON_OFF(d->manchester), ON_OFF(d->fec),
                       d->sync_mode);

  /* Packet control. */
  if (d->white_data != BOOL_INVAL)
    CC1101_REGS_REG(regs, PKTCTRL0).WHITE_DATA = d->white_data;
  if (d->pkt_format != UINT_MAX)
    CC1101_REGS_REG(regs, PKTCTRL0).PKT_FORMAT = d->pkt_format;
  if (d->crc != BOOL_INVAL) CC1101_REGS_REG(regs, PKTCTRL0).CRC_EN = d->crc;
  if (d->len_cfg != UINT_MAX)
    CC1101_REGS_REG(regs, PKTCTRL0).LENGTH_CONFIG = d->len_cfg;

#undef ri
  return true;
}

#define SET_CONF_FMT                                                 \
  "{"                                                                \
  "kbaud:%f,mhz:%f,dev_khz:%f,"                                      \
  "modulation:{format:%u,manchester:%B,fec:%B,sync_mode:%u},"        \
  "packet_control:{white_data:%B,format:%u,crc:%B,length_config:%u}" \
  "}"
static void cc1101_set_conf_handler(struct mg_rpc_request_info *ri,
                                    void *cb_arg, struct mg_rpc_frame_info *fi,
                                    struct mg_str args) {
  struct cc1101_set_conf_cb d = {
    ri : ri,
    kbaud : NAN,
    mhz : NAN,
    dev_khz : NAN,
    mod_format : UINT_MAX,
    manchester : BOOL_INVAL,
    fec : BOOL_INVAL,
    sync_mode : UINT_MAX,
    white_data : BOOL_INVAL,
    pkt_format : UINT_MAX,
    crc : BOOL_INVAL,
    len_cfg : UINT_MAX
  };
  if (json_scanf(args.p, args.len, ri->args_fmt, &d.kbaud, &d.mhz, &d.dev_khz,
                 &d.mod_format, &d.manchester, &d.fec, &d.sync_mode,
                 &d.white_data, &d.pkt_format, &d.crc, &d.len_cfg) < 1)
    mg_rpc_errorf_ret(400, "no settings passed");

  bool ok =
      mgos_cc1101_mod_regs(mgos_cc1101_get_global_locked(), CC1101_PKTCTRL0,
                           CC1101_DEVIATN, cc1101_set_conf_cb, &d);
  mgos_cc1101_put_global_locked();
  if (!ok && !d.ri) return;
  if (!ok) mg_rpc_errorf_ret(500, "error configuring CC1101 RF module");
  mg_rpc_send_responsef(ri, NULL);
}

#define TX_FMT "{data:%H,bits:%u,copies:%u}"
static void cc1101_tx_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                              struct mg_rpc_frame_info *fi,
                              struct mg_str args) {
  unsigned copies = 0, bits = UINT_MAX;
  uint8_t *data = NULL;
  int dataLen = 0;
  json_scanf(args.p, args.len, ri->args_fmt, &dataLen, &data, &bits, &copies);
  if (!data || !dataLen) mg_rpc_errorf_ret(400, "no data passed");
  if (bits == UINT_MAX) bits = dataLen * 8;
  if ((bits + 7) / 8 != dataLen)
    mg_rpc_errorf_ret(400, "%u bit%s makes no sense with %d data byte%s", bits,
                      MUL(bits), dataLen, MUL(dataLen));
  struct mgos_cc1101_tx_req req = {
    data : data,
    len : bits,
    copies : copies,
    free_data : true,
    cb : txop_save
  };
  txop.busy = true;
  bool ok = mgos_cc1101_tx(mgos_cc1101_get_global_locked(), &req);
  mgos_cc1101_put_global_locked();
  if (!ok) {
    txop.busy = false;
    free(data);
    mg_rpc_errorf_ret(500, "error sending %u bits %u time%s", bits, copies + 1,
                      MUL(copies + 1));
  }
  mg_rpc_send_responsef(ri, NULL);
}

#define DISTRIB_FMT "{min:%u,max:%u,avg:%u,cnt:%u}"
#define DISTRIB_ARGS(d) \
  d.min, d.max, !d.cnt ? 0 : (d.tot + d.cnt / 2) / d.cnt, d.cnt
static void cc1101_tx_stats_handler(struct mg_rpc_request_info *ri,
                                    void *cb_arg, struct mg_rpc_frame_info *fi,
                                    struct mg_str args) {
  mg_rpc_send_responsef(ri,
                        "{busy:%B,err:%d,delay_us:" DISTRIB_FMT
                        ",feed_us:" DISTRIB_FMT ",fifo_byt:" DISTRIB_FMT "}",
                        txop.busy, txop.err, DISTRIB_ARGS(txop.st.delay_us),
                        DISTRIB_ARGS(txop.st.feed_us),
                        DISTRIB_ARGS(txop.st.fifo_byt));
}

#define WRITE_REG_FMT "{reg:%u,val:%u}"
static void cc1101_write_reg_handler(struct mg_rpc_request_info *ri,
                                     void *cb_arg, struct mg_rpc_frame_info *fi,
                                     struct mg_str args) {
  unsigned reg, val;
  if (json_scanf(args.p, args.len, ri->args_fmt, &reg, &val) != 2)
    mg_rpc_errorf_ret(400, "reg and val are required");
  bool ok = mgos_cc1101_write_reg(mgos_cc1101_get_global_locked(), reg, val);
  mgos_cc1101_put_global_locked();
  if (!ok) mg_rpc_errorf_ret(500, "error writing register %u", reg);
  mg_rpc_send_responsef(ri, "{reg:%u,val:%u}", reg, val);
}

bool mgos_rpc_service_cc1101_init() {
  if (!mgos_sys_config_get_cc1101_rpc_enable() || !mgos_cc1101_get_global())
    return true;
  struct mg_rpc *rpc = mgos_rpc_get_global();
  mg_rpc_add_handler(rpc, "CC1101.Identify", "", cc1101_identify_handler, NULL);
  mg_rpc_add_handler(rpc, "CC1101.ReadReg", READ_REG_FMT,
                     cc1101_read_reg_handler, NULL);
  mg_rpc_add_handler(rpc, "CC1101.ReadRegs", READ_REGS_FMT,
                     cc1101_read_regs_handler, NULL);
  mg_rpc_add_handler(rpc, "CC1101.Reset", "", cc1101_reset_handler, NULL);
  mg_rpc_add_handler(rpc, "CC1101.SetConf", SET_CONF_FMT,
                     cc1101_set_conf_handler, NULL);
  mg_rpc_add_handler(rpc, "CC1101.Tx", TX_FMT, cc1101_tx_handler, NULL);
  mg_rpc_add_handler(rpc, "CC1101.TxStats", "", cc1101_tx_stats_handler, NULL);
  mg_rpc_add_handler(rpc, "CC1101.WriteReg", WRITE_REG_FMT,
                     cc1101_write_reg_handler, NULL);
  return true;
}
