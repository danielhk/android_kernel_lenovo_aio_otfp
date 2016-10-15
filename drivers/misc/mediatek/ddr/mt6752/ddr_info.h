#ifndef DDR_INFO_H
#define DDR_INFO_H

#define DDR_INFO_TAG                "[DDR INFO]"
#define ddr_info_warn(fmt, arg...)  pr_warn(DDR_INFO_TAG fmt, ##arg)
#define ddr_info_dbg(fmt, arg...)   pr_info(DDR_INFO_TAG fmt, ##arg)

#define GET_FIELD(var, mask, pos)     (((var) & (mask)) >> (pos))

 /**************************************************************************
 *  type define
 **************************************************************************/
 struct ddr_reg_base {
     void __iomem *ddrphy_base;
     void __iomem *dramc0_base;
     void __iomem *dramc_nao_base;
     void __iomem *apmixed_base;
 };
 
 struct ddr_info_driver {
     struct device_driver driver;
     struct ddr_reg_base reg_base;
     const struct platform_device_id *id_table;
 };


#endif /* end of DDR_INFO_H */

