//###########################################################################
//--------------------------------------------------------------------------
// FILE:Constant.h
// DESCRIPTION:
//         including those constants used in the 1140V SVG system
//--------------------------------------------------------------------------
// Author: Chao  Zhang
// Date: June  1, 2015
// Target System: DSP F28335 +　FPGA
// Version: V2.1	LI Shuzhen
//--------------------------------------------------------------------------
//###########################################################################
#ifndef SVG_CONSTANT_H
#define SVG_CONSTANT_H

//------------------------滤波常量-------------------------------
#define Delta_T 	0.00008	   //滤波时间常数,对应80us的中断周期
#define Delta_Tdc 	0.00048	   //Local_DC_Process中滤波的时间常数
//--------------------------模块数及对应周期数字量、载波计数值-------------------------//
#define DRAM_data   (int *) 0x100000   //Start Addr of XINTF Zone6
#define Nm            12          	//每相级联模块数
#define avg_Nm        0.0833333   	//12个模块平均分担
unsigned int period = 18000;     	//峰值
#define Half_period  9000      	 	//三角载波偏移量
#define Avg_N        750     	 	//平均每个模块承担1/3的电网电压,SPWM以DC为基准值标幺,还要考虑直流偏置是载波中点
#define ModuWaveLimit_Low   500		//调制波限幅（低）,该值的确定与死区时间、PWM开关周期和三角波的幅值有关。
#define ModuWaveLimit_High  17500	//调制波限幅（高）,该值的确定与死区时间、PWM开关周期和三角波的幅值有关。
//----------------------采样取平均-----------------------------//
#define UL2P	    	0.333333  	//将线电压转换为相电压,需乘以1/3,相量图分析
#define Sample_Time 	0.04  		//  1/25 = 0.04
#define No_Sample   	25  		//每个电网周期20ms采样 20/0.16=125个PCC点电压电流，每隔5个数存入数组,再取平均值，去除零漂， 125/5=25
//--------------------------状态机-------------------------//
#define Idle_State  0x00    		//闲置状态,ready等待状态
#define Soft_Start  0x01
#define Run_State   0x02
#define Stop_State  0x03
//--------------------------常用常量宏定义-------------------------//
#define	SQRT3		1.732			//sqrt(3)
#define	SQRT3d2		0.866           //sqrt(3)/2
#define	SQRT3_2		1.224745        //sqrt(3/2)
#define	SQRT2_3		0.8165          //sqrt(2/3)
#define	SQRT3d3		0.57735         //sqrt(3)/3
#define	SQRT2		1.41421356		//sqrt(2)
#define	SQRT2d2		0.7071068		//sqrt(2)/2
#define	PAI			3.14159265      //圆周率
//#define WL          0.52		//有功无功耦合项,1.65mH*100π
//------------------------接触器Switch---------------------------
#define  S_OFF      GpioDataRegs.GPADAT.bit.GPIO16 == 1
#define  S_ON       GpioDataRegs.GPADAT.bit.GPIO16 == 0

//------------------------485C_CON控制线-------------------------
#define  TXDC       GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
#define  RXDC       GpioDataRegs.GPBSET.bit.GPIO60 = 1

//------------------------上下限幅-------------------------------
#define Out_Limit(in, UpLim, LowLim)    (in>=UpLim)?UpLim:((in<=(LowLim))?(LowLim):in)

//----------------------初始化常量（100AC）------------------------
#define UDCMAX_initial      900
#define UDCMIN_initial      600
#define	UDCREF_initial		800
#define UDC_Kp_initial      10
#define UDC_Ki_initial      1
#define Ud_Kp_initial      	9
#define Ud_Ki_initial      	60
#define Kp_P2P_Neg_initial	2
#define Kp_in_initial		2
#define	Kp_Rpt_initial		1
#define Q_REF_initial       10
#define Ipcc_Amp_initial	3
#define PLL_Swtich_initial	1
#define THETA_COM_initial	2

//--------------------直流侧稳压控制变量------------------------
#define UDC_PI_Limit	150
#define Idref_Limit		150   //直流侧总有功电流限幅

//--------------------电流环用到的常量---------------------------
#define Curr_PI_Limit	400

//-------------------相间平衡控制常量----------------------------
#define  U_neg_amp_lim  -200
#define  I_neg_amp_lim  -20
#define  U_Z_amp_lim    300


//-------------------常数----------------------------------
#define C_COS60  0.5			//cos60
#define C_SIN60	 0.8660254		//sin60
#define C_32COEF 0.8164966		//sqrt(2/3)
#define	C_SQRT2	 1.414213		//sqrt(2)

#define Theta_SD  0.5
#define Theta_DB  11.3778			//计算角度用到的系数 1024/90==11.3777777

//#define DSP28_PLLCR    10
//#define DSP28_DIVSEL   2

//---------------------------FaultState故障显示------------------------------
#define 	LocalOverMAX    	0x0001  //直流侧电压超过限幅值
#define 	LocalUnderMIN   	0x0002  //直流侧电压低于限幅值
#define 	Conductor_Fault 	0x0004  //接触器返回故障
#define 	RD_EEPROM_OV7_ERROR 0x0008
#define 	WR_EEPROM_OV7_ERROR 0x0010  //EEPROM校验错误
#define 	IGBT_Fault_State 	0x0020  //分控上传，IGBT等故障
#define     Local_Fault         0x0040	//分控板返回的故障状态不是108
#define 	Node_Lost 	        0x0080  //某分控节点失联

//---------------------------ERROR_module标记-------------------------------
#define 	F1			0x0001
#define 	F2			0x0002
#define 	F3			0x0004
#define 	F4			0x0008
#define 	F5			0x0010
#define 	F6			0x0020
#define 	F7			0x0040
#define 	F8			0x0080
#define 	F9			0x0100
#define 	F10			0x0200
#define 	F11			0x0400
#define 	F12			0x0800
//-----------------------------1025个点-------------------------------------
const float SIN_TABLE[1025]={ 
0.0000000 ,0.0015340 ,0.0030680 ,0.0046019 ,0.0061359 ,0.0076698 ,0.0092038 
,0.0107377 ,0.0122715 ,0.0138054 ,0.0153392 ,0.0168730 ,0.0184067 ,0.0199404 
,0.0214741 ,0.0230077 ,0.0245412 ,0.0260747 ,0.0276081 ,0.0291415 ,0.0306748 
,0.0322080 ,0.0337412 ,0.0352742 ,0.0368072 ,0.0383401 ,0.0398729 ,0.0414056 
,0.0429383 ,0.0444708 ,0.0460032 ,0.0475355 ,0.0490677 ,0.0505997 ,0.0521317 
,0.0536635 ,0.0551952 ,0.0567268 ,0.0582583 ,0.0597896 ,0.0613207 ,0.0628518 
,0.0643826 ,0.0659134 ,0.0674439 ,0.0689743 ,0.0705046 ,0.0720347 ,0.0735646 
,0.0750943 ,0.0766239 ,0.0781532 ,0.0796824 ,0.0812114 ,0.0827403 ,0.0842689 
,0.0857973 ,0.0873255 ,0.0888536 ,0.0903814 ,0.0919090 ,0.0934363 ,0.0949635 
,0.0964904 ,0.0980171 ,0.0995436 ,0.1010699 ,0.1025959 ,0.1041216 ,0.1056472 
,0.1071724 ,0.1086974 ,0.1102222 ,0.1117467 ,0.1132710 ,0.1147949 ,0.1163186 
,0.1178421 ,0.1193652 ,0.1208881 ,0.1224107 ,0.1239330 ,0.1254550 ,0.1269767 
,0.1284981 ,0.1300192 ,0.1315400 ,0.1330605 ,0.1345807 ,0.1361006 ,0.1376201 
,0.1391393 ,0.1406582 ,0.1421768 ,0.1436950 ,0.1452129 ,0.1467305 ,0.1482477 
,0.1497645 ,0.1512810 ,0.1527972 ,0.1543130 ,0.1558284 ,0.1573435 ,0.1588581 
,0.1603725 ,0.1618864 ,0.1633999 ,0.1649131 ,0.1664259 ,0.1679383 ,0.1694503 
,0.1709619 ,0.1724731 ,0.1739839 ,0.1754943 ,0.1770042 ,0.1785138 ,0.1800229 
,0.1815316 ,0.1830399 ,0.1845477 ,0.1860552 ,0.1875621 ,0.1890687 ,0.1905748 
,0.1920804 ,0.1935856 ,0.1950903 ,0.1965946 ,0.1980984 ,0.1996018 ,0.2011046 
,0.2026070 ,0.2041090 ,0.2056104 ,0.2071114 ,0.2086119 ,0.2101118 ,0.2116113 
,0.2131103 ,0.2146088 ,0.2161068 ,0.2176043 ,0.2191012 ,0.2205977 ,0.2220936 
,0.2235890 ,0.2250839 ,0.2265783 ,0.2280721 ,0.2295654 ,0.2310581 ,0.2325503 
,0.2340420 ,0.2355331 ,0.2370236 ,0.2385136 ,0.2400030 ,0.2414919 ,0.2429802 
,0.2444679 ,0.2459550 ,0.2474416 ,0.2489276 ,0.2504130 ,0.2518978 ,0.2533820 
,0.2548657 ,0.2563487 ,0.2578311 ,0.2593129 ,0.2607941 ,0.2622747 ,0.2637547 
,0.2652340 ,0.2667128 ,0.2681909 ,0.2696683 ,0.2711452 ,0.2726214 ,0.2740969 
,0.2755718 ,0.2770461 ,0.2785197 ,0.2799926 ,0.2814649 ,0.2829366 ,0.2844075 
,0.2858778 ,0.2873475 ,0.2888164 ,0.2902847 ,0.2917523 ,0.2932192 ,0.2946854 
,0.2961509 ,0.2976157 ,0.2990798 ,0.3005432 ,0.3020059 ,0.3034679 ,0.3049292 
,0.3063898 ,0.3078496 ,0.3093088 ,0.3107672 ,0.3122248 ,0.3136817 ,0.3151379 
,0.3165934 ,0.3180481 ,0.3195020 ,0.3209552 ,0.3224077 ,0.3238594 ,0.3253103 
,0.3267604 ,0.3282098 ,0.3296585 ,0.3311063 ,0.3325534 ,0.3339996 ,0.3354451 
,0.3368899 ,0.3383338 ,0.3397769 ,0.3412192 ,0.3426607 ,0.3441014 ,0.3455413 
,0.3469804 ,0.3484187 ,0.3498561 ,0.3512928 ,0.3527286 ,0.3541635 ,0.3555977 
,0.3570310 ,0.3584634 ,0.3598950 ,0.3613258 ,0.3627557 ,0.3641848 ,0.3656130 
,0.3670403 ,0.3684668 ,0.3698924 ,0.3713172 ,0.3727411 ,0.3741641 ,0.3755862 
,0.3770074 ,0.3784278 ,0.3798472 ,0.3812658 ,0.3826834 ,0.3841002 ,0.3855161 
,0.3869310 ,0.3883450 ,0.3897582 ,0.3911704 ,0.3925817 ,0.3939920 ,0.3954015 
,0.3968100 ,0.3982176 ,0.3996242 ,0.4010299 ,0.4024346 ,0.4038385 ,0.4052413 
,0.4066432 ,0.4080442 ,0.4094441 ,0.4108432 ,0.4122412 ,0.4136383 ,0.4150344 
,0.4164296 ,0.4178237 ,0.4192169 ,0.4206091 ,0.4220003 ,0.4233905 ,0.4247797 
,0.4261679 ,0.4275551 ,0.4289413 ,0.4303265 ,0.4317107 ,0.4330938 ,0.4344760 
,0.4358571 ,0.4372372 ,0.4386162 ,0.4399943 ,0.4413713 ,0.4427472 ,0.4441221 
,0.4454960 ,0.4468688 ,0.4482406 ,0.4496113 ,0.4509810 ,0.4523496 ,0.4537171 
,0.4550836 ,0.4564490 ,0.4578133 ,0.4591765 ,0.4605387 ,0.4618998 ,0.4632598 
,0.4646187 ,0.4659765 ,0.4673332 ,0.4686888 ,0.4700433 ,0.4713967 ,0.4727490 
,0.4741002 ,0.4754503 ,0.4767992 ,0.4781471 ,0.4794938 ,0.4808393 ,0.4821838 
,0.4835271 ,0.4848692 ,0.4862103 ,0.4875502 ,0.4888889 ,0.4902265 ,0.4915629 
,0.4928982 ,0.4942323 ,0.4955653 ,0.4968970 ,0.4982277 ,0.4995571 ,0.5008854 
,0.5022125 ,0.5035384 ,0.5048631 ,0.5061866 ,0.5075090 ,0.5088301 ,0.5101501 
,0.5114688 ,0.5127864 ,0.5141027 ,0.5154179 ,0.5167318 ,0.5180445 ,0.5193560 
,0.5206663 ,0.5219753 ,0.5232831 ,0.5245897 ,0.5258950 ,0.5271991 ,0.5285020 
,0.5298036 ,0.5311040 ,0.5324031 ,0.5337010 ,0.5349976 ,0.5362930 ,0.5375871 
,0.5388799 ,0.5401715 ,0.5414618 ,0.5427508 ,0.5440385 ,0.5453250 ,0.5466102 
,0.5478941 ,0.5491767 ,0.5504580 ,0.5517380 ,0.5530167 ,0.5542941 ,0.5555702 
,0.5568450 ,0.5581185 ,0.5593907 ,0.5606616 ,0.5619311 ,0.5631993 ,0.5644662 
,0.5657318 ,0.5669960 ,0.5682589 ,0.5695205 ,0.5707807 ,0.5720396 ,0.5732972 
,0.5745534 ,0.5758082 ,0.5770617 ,0.5783138 ,0.5795646 ,0.5808140 ,0.5820620 
,0.5833086 ,0.5845539 ,0.5857979 ,0.5870404 ,0.5882815 ,0.5895213 ,0.5907597 
,0.5919967 ,0.5932323 ,0.5944665 ,0.5956993 ,0.5969307 ,0.5981607 ,0.5993893 
,0.6006165 ,0.6018422 ,0.6030666 ,0.6042895 ,0.6055110 ,0.6067311 ,0.6079498 
,0.6091670 ,0.6103828 ,0.6115972 ,0.6128101 ,0.6140216 ,0.6152316 ,0.6164402 
,0.6176473 ,0.6188530 ,0.6200572 ,0.6212600 ,0.6224613 ,0.6236611 ,0.6248595 
,0.6260564 ,0.6272518 ,0.6284458 ,0.6296382 ,0.6308292 ,0.6320187 ,0.6332068 
,0.6343933 ,0.6355783 ,0.6367619 ,0.6379439 ,0.6391244 ,0.6403035 ,0.6414810 
,0.6426570 ,0.6438315 ,0.6450045 ,0.6461760 ,0.6473460 ,0.6485144 ,0.6496813 
,0.6508467 ,0.6520105 ,0.6531728 ,0.6543336 ,0.6554928 ,0.6566505 ,0.6578067 
,0.6589613 ,0.6601143 ,0.6612658 ,0.6624158 ,0.6635642 ,0.6647110 ,0.6658562 
,0.6669999 ,0.6681420 ,0.6692826 ,0.6704216 ,0.6715590 ,0.6726948 ,0.6738290 
,0.6749616 ,0.6760927 ,0.6772222 ,0.6783500 ,0.6794763 ,0.6806010 ,0.6817241 
,0.6828455 ,0.6839654 ,0.6850837 ,0.6862003 ,0.6873153 ,0.6884287 ,0.6895405 
,0.6906507 ,0.6917593 ,0.6928662 ,0.6939715 ,0.6950751 ,0.6961771 ,0.6972775 
,0.6983762 ,0.6994733 ,0.7005688 ,0.7016626 ,0.7027547 ,0.7038452 ,0.7049341 
,0.7060213 ,0.7071068 ,0.7081906 ,0.7092728 ,0.7103533 ,0.7114322 ,0.7125094 
,0.7135849 ,0.7146587 ,0.7157308 ,0.7168013 ,0.7178700 ,0.7189371 ,0.7200025 
,0.7210662 ,0.7221282 ,0.7231885 ,0.7242471 ,0.7253040 ,0.7263592 ,0.7274126 
,0.7284644 ,0.7295144 ,0.7305628 ,0.7316094 ,0.7326543 ,0.7336974 ,0.7347389 
,0.7357786 ,0.7368166 ,0.7378528 ,0.7388873 ,0.7399201 ,0.7409511 ,0.7419804 
,0.7430079 ,0.7440337 ,0.7450578 ,0.7460801 ,0.7471006 ,0.7481194 ,0.7491364 
,0.7501516 ,0.7511651 ,0.7521768 ,0.7531868 ,0.7541950 ,0.7552014 ,0.7562060 
,0.7572088 ,0.7582099 ,0.7592092 ,0.7602067 ,0.7612024 ,0.7621963 ,0.7631884 
,0.7641787 ,0.7651673 ,0.7661540 ,0.7671389 ,0.7681220 ,0.7691033 ,0.7700828 
,0.7710605 ,0.7720364 ,0.7730104 ,0.7739827 ,0.7749531 ,0.7759217 ,0.7768885 
,0.7778534 ,0.7788165 ,0.7797778 ,0.7807372 ,0.7816948 ,0.7826506 ,0.7836045 
,0.7845566 ,0.7855068 ,0.7864552 ,0.7874017 ,0.7883464 ,0.7892892 ,0.7902302 
,0.7911693 ,0.7921066 ,0.7930420 ,0.7939755 ,0.7949071 ,0.7958369 ,0.7967648 
,0.7976908 ,0.7986150 ,0.7995373 ,0.8004577 ,0.8013762 ,0.8022928 ,0.8032075 
,0.8041204 ,0.8050313 ,0.8059404 ,0.8068475 ,0.8077528 ,0.8086562 ,0.8095576 
,0.8104572 ,0.8113548 ,0.8122506 ,0.8131444 ,0.8140363 ,0.8149263 ,0.8158144 
,0.8167006 ,0.8175848 ,0.8184671 ,0.8193475 ,0.8202260 ,0.8211025 ,0.8219771 
,0.8228498 ,0.8237205 ,0.8245893 ,0.8254561 ,0.8263211 ,0.8271840 ,0.8280450 
,0.8289041 ,0.8297612 ,0.8306164 ,0.8314696 ,0.8323209 ,0.8331702 ,0.8340175 
,0.8348629 ,0.8357063 ,0.8365477 ,0.8373872 ,0.8382247 ,0.8390602 ,0.8398938 
,0.8407254 ,0.8415550 ,0.8423826 ,0.8432082 ,0.8440319 ,0.8448536 ,0.8456732 
,0.8464909 ,0.8473066 ,0.8481203 ,0.8489321 ,0.8497418 ,0.8505495 ,0.8513552 
,0.8521589 ,0.8529606 ,0.8537603 ,0.8545580 ,0.8553537 ,0.8561473 ,0.8569390 
,0.8577286 ,0.8585162 ,0.8593018 ,0.8600854 ,0.8608669 ,0.8616465 ,0.8624240 
,0.8631994 ,0.8639729 ,0.8647443 ,0.8655136 ,0.8662809 ,0.8670462 ,0.8678095 
,0.8685707 ,0.8693299 ,0.8700870 ,0.8708421 ,0.8715951 ,0.8723461 ,0.8730950 
,0.8738418 ,0.8745866 ,0.8753294 ,0.8760701 ,0.8768087 ,0.8775453 ,0.8782798 
,0.8790122 ,0.8797426 ,0.8804709 ,0.8811971 ,0.8819213 ,0.8826433 ,0.8833633 
,0.8840813 ,0.8847971 ,0.8855109 ,0.8862225 ,0.8869321 ,0.8876396 ,0.8883450 
,0.8890484 ,0.8897496 ,0.8904487 ,0.8911458 ,0.8918407 ,0.8925336 ,0.8932243 
,0.8939129 ,0.8945995 ,0.8952839 ,0.8959662 ,0.8966465 ,0.8973246 ,0.8980006 
,0.8986745 ,0.8993462 ,0.9000159 ,0.9006834 ,0.9013488 ,0.9020121 ,0.9026733 
,0.9033324 ,0.9039893 ,0.9046441 ,0.9052968 ,0.9059473 ,0.9065957 ,0.9072420 
,0.9078861 ,0.9085281 ,0.9091680 ,0.9098057 ,0.9104413 ,0.9110747 ,0.9117060 
,0.9123352 ,0.9129622 ,0.9135870 ,0.9142098 ,0.9148303 ,0.9154487 ,0.9160650 
,0.9166791 ,0.9172910 ,0.9179008 ,0.9185084 ,0.9191138 ,0.9197171 ,0.9203183 
,0.9209172 ,0.9215140 ,0.9221087 ,0.9227011 ,0.9232914 ,0.9238795 ,0.9244655 
,0.9250492 ,0.9256308 ,0.9262102 ,0.9267875 ,0.9273625 ,0.9279354 ,0.9285061 
,0.9290746 ,0.9296409 ,0.9302050 ,0.9307670 ,0.9313267 ,0.9318843 ,0.9324396 
,0.9329928 ,0.9335438 ,0.9340925 ,0.9346391 ,0.9351835 ,0.9357257 ,0.9362657 
,0.9368034 ,0.9373390 ,0.9378724 ,0.9384035 ,0.9389325 ,0.9394592 ,0.9399837 
,0.9405061 ,0.9410262 ,0.9415441 ,0.9420597 ,0.9425732 ,0.9430844 ,0.9435935 
,0.9441003 ,0.9446048 ,0.9451072 ,0.9456073 ,0.9461052 ,0.9466009 ,0.9470944 
,0.9475856 ,0.9480746 ,0.9485613 ,0.9490459 ,0.9495282 ,0.9500082 ,0.9504861 
,0.9509617 ,0.9514350 ,0.9519061 ,0.9523750 ,0.9528416 ,0.9533060 ,0.9537682 
,0.9542281 ,0.9546858 ,0.9551412 ,0.9555943 ,0.9560452 ,0.9564939 ,0.9569403 
,0.9573845 ,0.9578264 ,0.9582661 ,0.9587035 ,0.9591386 ,0.9595715 ,0.9600021 
,0.9604305 ,0.9608566 ,0.9612805 ,0.9617021 ,0.9621214 ,0.9625385 ,0.9629533 
,0.9633658 ,0.9637761 ,0.9641841 ,0.9645898 ,0.9649932 ,0.9653944 ,0.9657934 
,0.9661900 ,0.9665844 ,0.9669765 ,0.9673663 ,0.9677538 ,0.9681391 ,0.9685221 
,0.9689028 ,0.9692812 ,0.9696574 ,0.9700313 ,0.9704028 ,0.9707721 ,0.9711392 
,0.9715039 ,0.9718663 ,0.9722265 ,0.9725844 ,0.9729399 ,0.9732932 ,0.9736442 
,0.9739930 ,0.9743394 ,0.9746835 ,0.9750253 ,0.9753649 ,0.9757021 ,0.9760371 
,0.9763697 ,0.9767001 ,0.9770281 ,0.9773539 ,0.9776774 ,0.9779985 ,0.9783174 
,0.9786339 ,0.9789482 ,0.9792601 ,0.9795698 ,0.9798771 ,0.9801821 ,0.9804849 
,0.9807853 ,0.9810834 ,0.9813792 ,0.9816727 ,0.9819639 ,0.9822527 ,0.9825393 
,0.9828235 ,0.9831055 ,0.9833851 ,0.9836624 ,0.9839374 ,0.9842101 ,0.9844805 
,0.9847485 ,0.9850142 ,0.9852776 ,0.9855387 ,0.9857975 ,0.9860540 ,0.9863081 
,0.9865599 ,0.9868094 ,0.9870566 ,0.9873014 ,0.9875439 ,0.9877841 ,0.9880220 
,0.9882576 ,0.9884908 ,0.9887217 ,0.9889503 ,0.9891765 ,0.9894004 ,0.9896220 
,0.9898413 ,0.9900582 ,0.9902728 ,0.9904851 ,0.9906950 ,0.9909026 ,0.9911079 
,0.9913109 ,0.9915115 ,0.9917098 ,0.9919057 ,0.9920993 ,0.9922906 ,0.9924795 
,0.9926661 ,0.9928504 ,0.9930323 ,0.9932119 ,0.9933892 ,0.9935641 ,0.9937367 
,0.9939070 ,0.9940749 ,0.9942404 ,0.9944037 ,0.9945646 ,0.9947231 ,0.9948793 
,0.9950332 ,0.9951847 ,0.9953339 ,0.9954808 ,0.9956253 ,0.9957674 ,0.9959072 
,0.9960447 ,0.9961798 ,0.9963126 ,0.9964431 ,0.9965711 ,0.9966969 ,0.9968203 
,0.9969414 ,0.9970601 ,0.9971764 ,0.9972905 ,0.9974021 ,0.9975115 ,0.9976184 
,0.9977231 ,0.9978253 ,0.9979253 ,0.9980229 ,0.9981181 ,0.9982110 ,0.9983015 
,0.9983897 ,0.9984756 ,0.9985591 ,0.9986402 ,0.9987190 ,0.9987955 ,0.9988695 
,0.9989413 ,0.9990107 ,0.9990777 ,0.9991424 ,0.9992048 ,0.9992647 ,0.9993224 
,0.9993777 ,0.9994306 ,0.9994812 ,0.9995294 ,0.9995753 ,0.9996188 ,0.9996600 
,0.9996988 ,0.9997353 ,0.9997694 ,0.9998012 ,0.9998306 ,0.9998576 ,0.9998823 
,0.9999047 ,0.9999247 ,0.9999423 ,0.9999576 ,0.9999706 ,0.9999812 ,0.9999894 
,0.9999953 ,0.9999988 ,1.0000000 
} ;

/******************************0-1对应0-45度,分为256个点************************************/
const float ARCTG_TABLE[257]={
0.000000 ,0.223811 ,0.447614 ,0.671404 ,0.895174 ,1.118916 ,1.342624 ,1.566291 
,1.789911 ,2.013476 ,2.236979 ,2.460415 ,2.683775 ,2.907054 ,3.130245 ,3.353341 
,3.576334 ,3.799220 ,4.021990 ,4.244639 ,4.467159 ,4.689544 ,4.911788 ,5.133884 
,5.355825 ,5.577605 ,5.799218 ,6.020656 ,6.241914 ,6.462986 ,6.683864 ,6.904543 
,7.125016 ,7.345278 ,7.565321 ,7.785140 ,8.004729 ,8.224081 ,8.443191 ,8.662052 
,8.880659 ,9.099006 ,9.317086 ,9.534894 ,9.752425 ,9.969672 ,10.186630 
,10.403293 ,10.619655 ,10.835712 ,11.051457 ,11.266885 ,11.481991 ,11.696770 
,11.911216 ,12.125323 ,12.339087 ,12.552503 ,12.765566 ,12.978270 ,13.190611 
,13.402583 ,13.614183 ,13.825405 ,14.036244 ,14.246695 ,14.456756 ,14.666419 
,14.875682 ,15.084540 ,15.292988 ,15.501022 ,15.708638 ,15.915832 ,16.122599 
,16.328936 ,16.534838 ,16.740302 ,16.945324 ,17.149899 ,17.354025 ,17.557697 
,17.760912 ,17.963666 ,18.165957 ,18.367779 ,18.569131 ,18.770008 ,18.970408 
,19.170327 ,19.369762 ,19.568710 ,19.767169 ,19.965134 ,20.162604 ,20.359575 
,20.556045 ,20.752011 ,20.947471 ,21.142421 ,21.336859 ,21.530784 ,21.724192 
,21.917081 ,22.109449 ,22.301293 ,22.492612 ,22.683404 ,22.873665 ,23.063396 
,23.252592 ,23.441254 ,23.629378 ,23.816963 ,24.004008 ,24.190510 ,24.376469 
,24.561882 ,24.746748 ,24.931067 ,25.114835 ,25.298053 ,25.480718 ,25.662830 
,25.844388 ,26.025390 ,26.205835 ,26.385722 ,26.565051 ,26.743821 ,26.922030 
,27.099678 ,27.276764 ,27.453287 ,27.629247 ,27.804643 ,27.979475 ,28.153741 
,28.327442 ,28.500578 ,28.673147 ,28.845149 ,29.016585 ,29.187453 ,29.357754 
,29.527487 ,29.696653 ,29.865251 ,30.033281 ,30.200743 ,30.367637 ,30.533964 
,30.699723 ,30.864914 ,31.029538 ,31.193595 ,31.357085 ,31.520009 ,31.682366 
,31.844158 ,32.005383 ,32.166044 ,32.326140 ,32.485672 ,32.644640 ,32.803046 
,32.960888 ,33.118169 ,33.274888 ,33.431047 ,33.586646 ,33.741686 ,33.896167 
,34.050090 ,34.203457 ,34.356268 ,34.508523 ,34.660224 ,34.811372 ,34.961968 
,35.112011 ,35.261505 ,35.410449 ,35.558844 ,35.706692 ,35.853993 ,36.000749 
,36.146961 ,36.292630 ,36.437757 ,36.582343 ,36.726390 ,36.869898 ,37.012869 
,37.155304 ,37.297205 ,37.438572 ,37.579407 ,37.719711 ,37.859486 ,37.998733 
,38.137453 ,38.275647 ,38.413318 ,38.550466 ,38.687092 ,38.823199 ,38.958788 
,39.093859 ,39.228415 ,39.362457 ,39.495987 ,39.629006 ,39.761515 ,39.893516 
,40.025010 ,40.156000 ,40.286486 ,40.416471 ,40.545955 ,40.674940 ,40.803428 
,40.931421 ,41.058919 ,41.185926 ,41.312441 ,41.438467 ,41.564006 ,41.689059 
,41.813627 ,41.937713 ,42.061318 ,42.184444 ,42.307091 ,42.429263 ,42.550961 
,42.672185 ,42.792939 ,42.913223 ,43.033040 ,43.152390 ,43.271276 ,43.389700 
,43.507662 ,43.625166 ,43.742211 ,43.858801 ,43.974937 ,44.090620 ,44.205852 
,44.320636 ,44.434972 ,44.548862 ,44.662308 ,44.775312 ,44.887876 ,45.000000 
} ;  	

//FPGA映射寄存器地址

//----------------------------全局命令--------------------------//
#define Sample           0x342      //向该地址写入任何数据都可以启动主控AD采样
//#define Trigger          0x20344      //载波同步计数触发
//这三个CMD无效,是纵向的,与横向并行分组矛盾
//#define Req_A_DC_Voltage 0x046      //当主节点发送该命令标识时，所有A相从节点向主节点回传母线电压
//#define Req_B_DC_Voltage 0x048      //当主节点发送该命令标识时，所有B相从节点向主节点回传母线电压
//#define Req_C_DC_Voltage 0x04A      //当主节点发送该命令标识时，所有C相从节点向主节点回传母线电压
//集中式控制,全局写指令,表示主控向分控下发数据
#define Req_DC_Voltage   0x20346    //当主节点发送该命令标识时，所有从节点向主节点回传母线电压
#define Req_State        0x20348    //当主节点发送该命令标识时，所有从节点返回故障状态
#define Run              0x2034C    //同步开通
#define State_Reset      0x20350    //故障状态同步复位
#define Local_Sample     0x20352    //各分控节点同步采样
#define Switch_ON		 0x354      //接触器合闸信号,即旁路软启电阻
#define Switch_OFF       0x356      //接触器分闸信号,即接入电阻
#define Shut             0x2038E	//全局PWM关断
#define Datas_Check      0x2034A	//全局数据链路检测
#define Carrier_Period   0x20344	//设置分控载波周期
#define Error_Check_A    0x20794	//Commu_Error A相
#define Error_Check_B    0x20B94	//Commu_Error B相
#define Error_Check_C    0x20F94	//Commu_Error C相
//-----------------------全局命令结束-------------------------------------//

//-------------------------主控命令--------------------------------------//
//主控板DA
#define DA_Main_Addr 0x00E       //向该地址写入数据会启动主控制器DA,设置DA保护阈值
//主控板AD
#define AD_Load_IA_Addr 0x00C  //读取该地址会获取主控板AD转换结果
#define AD_Load_IB_Addr 0x00A  //读取该地址会获取主控板AD转换结果
#define AD_Load_IC_Addr 0x008  //读取该地址会获取主控板AD转换结果
#define AD_PCC_VA_Addr 0x006  //读取该地址会获取主控板AD转换结果
#define AD_PCC_VB_Addr 0x004  //读取该地址会获取主控板AD转换结果
#define AD_PCC_VC_Addr 0x002  //读取该地址会获取主控板AD转换结果

#define AD_INV_IC_Addr 0x010  //读取该地址会获取主控板AD转换结果
#define AD_INV_IB_Addr 0x012  //读取该地址会获取主控板AD转换结果
#define AD_INV_IA_Addr 0x014  //读取该地址会获取主控板AD转换结果
#define AD_Rsvd_VC_Addr 0x016  //读取该地址会获取主控板AD转换结果
#define AD_Rsvd_VB_Addr 0x018  //读取该地址会获取主控板AD转换结果
#define AD_Rsvd_VA_Addr 0x01A  //读取该地址会获取主控板AD转换结果
/************************主控命令结束********************************/

//XA17=1作为FPGA_PWM的片选信号
/************************A相命令************************************/
//节点1
//A1 PWM
#define Modulation_A1_Addr  0x20442
#define Carrier_A1_Addr     0x20444
#define Direction_A1_Addr   0x20446
#define InialCnt_A1_Addr    0x20448
//A1 AD
#define AD_A1_DC_Addr  0x2044A   //向该地址写入任何数据，表示让A1单元传回DC电压。读该地址数据为读取A1的DC电压
#define AD_A1_BV_Addr  0x2044C
#define AD_A1_BC_Addr  0x2044E
//A1 STATE
#define State_A1_Addr  0x20450 //向该地址写入任何数据，表示让A1单元传回故障信息。读该地址数据为读取A1的故障信息
//数据收发测试
#define Send_Check_A1_Addr  0x20452
#define Data_Check_A1_Addr 0x20454
//A1 左右桥臂载波计数归零
#define L_Carrier_A1_Addr  0x20456
#define R_Carrier_A1_Addr  0x20458

//节点2
//A2 PWM
#define Modulation_A2_Addr  0x20482
#define Carrier_A2_Addr     0x20484  //载波周期计数值
#define Direction_A2_Addr   0x20486
#define InialCnt_A2_Addr    0x20488
//A2 AD
#define AD_A2_DC_Addr  0x2048A   //向该地址写入任何数据，表示让A2单元传回DC电压。读该地址数据为读取A2的DC电压
#define AD_A2_BV_Addr  0x2048C
#define AD_A2_BC_Addr  0x2048E
//A2 STATE
#define State_A2_Addr  0x20490 //向该地址写入任何数据，表示让A2单元传回故障信息。读该地址数据为读取A2的故障信息
//数据收发测试
#define Send_Check_A2_Addr 0x20492
#define Data_Check_A2_Addr 0x20494
//A2 左右桥臂载波计数归零
#define L_Carrier_A2_Addr  0x20496
#define R_Carrier_A2_Addr  0x20498

//节点3
//A3 PWM
#define Modulation_A3_Addr  0x204C2
#define Carrier_A3_Addr     0x204C4
#define Direction_A3_Addr   0x204C6
#define InialCnt_A3_Addr    0x204C8
//A3 AD
#define AD_A3_DC_Addr  0x204CA   //向该地址写入任何数据，表示让A3单元传回DC电压。读该地址数据为读取A3的DC电压
#define AD_A3_BV_Addr  0x204CC
#define AD_A3_BC_Addr  0x204CE
//A3 STATE
#define State_A3_Addr  0x204D0 //向该地址写入任何数据，表示让A3单元传回故障信息。读该地址数据为读取A3的故障信息
//数据收发测试
#define Send_Check_A3_Addr  0x204D2
#define Data_Check_A3_Addr  0x204D4
//A3 左右桥臂载波计数归零
#define L_Carrier_A3_Addr  0x204D6
#define R_Carrier_A3_Addr  0x204D8

//节点4
//A4 PWM
#define Modulation_A4_Addr  0x20502
#define Carrier_A4_Addr     0x20504
#define Direction_A4_Addr   0x20506
#define InialCnt_A4_Addr    0x20508
//A4 AD
#define AD_A4_DC_Addr  0x2050A   //向该地址写入任何荩硎救肁4单元传回DC电压。读该地址数据为读取A4的DC电压
#define AD_A4_BV_Addr  0x2050C
#define AD_A4_BC_Addr  0x2050E
//A4 STATE
#define State_A4_Addr  0x20510 //向该地址写入任何数据，表示让A4单元传回故障信息。读该地址数据为读取A4的故障信息
//数据收发测试
#define Send_Check_A4_Addr  0x20512
#define Data_Check_A4_Addr  0x20514
//A4 左右桥臂载波计数归零
#define L_Carrier_A4_Addr  0x20516
#define R_Carrier_A4_Addr  0x20518

//节点5
//A5 PWM
#define Modulation_A5_Addr  0x20542
#define Carrier_A5_Addr     0x20544
#define Direction_A5_Addr   0x20546
#define InialCnt_A5_Addr    0x20548
//A5 AD
#define AD_A5_DC_Addr  0x2054A   //向该地址写入任何数据，表示让A5单元传回DC电压。读该地址数据为读取A5的DC电压
#define AD_A5_BV_Addr  0x2054C
#define AD_A5_BC_Addr  0x2054E
//A5 STATE
#define State_A5_Addr  0x20550 //向该地址写入任何数据，表示让A5单元传回故障信息。读该地址数据为读取A5的故障信息
//数据收发测试
#define Send_Check_A5_Addr  0x20552
#define Data_Check_A5_Addr  0x20554
//A5 左右桥臂载波计数归零
#define L_Carrier_A5_Addr  0x20556
#define R_Carrier_A5_Addr  0x20558

//节点6
//A6 PWM
#define Modulation_A6_Addr  0x20582
#define Carrier_A6_Addr     0x20584
#define Direction_A6_Addr   0x20586
#define InialCnt_A6_Addr    0x20588
//A6 AD
#define AD_A6_DC_Addr  0x2058A   //向该地址写入任何数据，表示让A6单元传回DC电压。读该地址数据为读取A6的DC电压
#define AD_A6_BV_Addr  0x2058C
#define AD_A6_BC_Addr  0x2058E
//A6 STATE
#define State_A6_Addr  0x20590 //向该地址写入任何数据，表示让A6单元传回故障信息。读该地址数据为读取A6的故障信息
//数据收发测试
#define Send_Check_A6_Addr  0x20592
#define Data_Check_A6_Addr 0x20594
//A6 左右桥臂载波计数归零
#define L_Carrier_A6_Addr  0x20596
#define R_Carrier_A6_Addr  0x20598

//节点7
//A7 PWM
#define Modulation_A7_Addr  0x205C2
#define Carrier_A7_Addr     0x205C4
#define Direction_A7_Addr   0x205C6
#define InialCnt_A7_Addr    0x205C8
//A7 AD
#define AD_A7_DC_Addr  0x205CA   //向该地址写入任何数据，表示让A7单元传回DC电压。读该地址数据为读取A7的DC电压
#define AD_A7_BV_Addr  0x205CC
#define AD_A7_BC_Addr  0x205CE
//A7 STATE
#define State_A7_Addr  0x205D0 //向该地址写入任何数据，表示让A7单元传回故障信息。读该地址数据为读取A7的故障信息
//A7 DA
#define DA_A7_Addr  0x205D2     //向该地址写入数据会启动A7的DA
//数据收发测试
#define Send_Check_A7_Addr 0x205D2
#define Data_Check_A7_Addr 0x205D4
//A7 左右桥臂载波计数归零
#define L_Carrier_A7_Addr  0x205D6
#define R_Carrier_A7_Addr  0x205D8

//节点8
//A8 PWM
#define Modulation_A8_Addr  0x20602
#define Carrier_A8_Addr     0x20604
#define Direction_A8_Addr   0x20606
#define InialCnt_A8_Addr    0x20608
//A8 AD
#define AD_A8_DC_Addr  0x2060A   //向该地址写入任何数据，表示让A8单元传回DC电压。读该地址数据为读取A8的DC电压
#define AD_A8_BV_Addr  0x2060C
#define AD_A8_BC_Addr  0x2060E
//A8 STATE
#define State_A8_Addr  0x20610 //向该地址写入任何数据，表示让A8单元传回故障信息。读该地址数据为读取A8的故障信息
//A8 DA
#define DA_A8_Addr  0x20612     //向该地址写入数据会启动A8的DA
//数据收发测试
#define Send_Check_A8_Addr 0x20612
#define Data_Check_A8_Addr 0x20614
//A8 左右桥臂载波计数归零
#define L_Carrier_A8_Addr  0x20616
#define R_Carrier_A8_Addr  0x20618

//节点9
//A9 PWM
#define Modulation_A9_Addr  0x20642
#define Carrier_A9_Addr     0x20644
#define Direction_A9_Addr   0x20646
#define InialCnt_A9_Addr    0x20648
//A9 AD
#define AD_A9_DC_Addr  0x2064A   //向该地址写入任何数据，表示让A9单元传回DC电压。读该地址数据为读取A9的DC电压
#define AD_A9_BV_Addr  0x2064C
#define AD_A9_BC_Addr  0x2064E
//A9 STATE
#define State_A9_Addr  0x20650 //向该地址写入任何数据，表示让A9单元传回故障信息。读该地址数据为读取A9的故障信息
//A9 DA
#define DA_A9_Addr  0x20652     //向该地址写入数据会启动A9的DA
//数据收发测试
#define Send_Check_A9_Addr 0x20652
#define Data_Check_A9_Addr 0x20654
//A9 左右桥臂载波计数归零
#define L_Carrier_A9_Addr  0x20656
#define R_Carrier_A9_Addr  0x20658

//节点10
//A10 PWM
#define Modulation_A10_Addr  0x20682
#define Carrier_A10_Addr     0x20684
#define Direction_A10_Addr   0x20686
#define InialCnt_A10_Addr    0x20688
//A10 AD
#define AD_A10_DC_Addr  0x2068A   //向该地址写入任何数据，表示让A10单元传回DC电压。读该地址数据为读取A10的DC电压
#define AD_A10_BV_Addr  0x2068C
#define AD_A10_BC_Addr  0x2068E
//A10 STATE
#define State_A10_Addr  0x20690 //向该地址写入任何数据，表示让A10单元传回故障信息。读该地址数据为读取A10的故障信息
//A10 DA
#define DA_A10_Addr  0x20692     //向该地址写入数据会启动A10的DA
//数据收发测试
#define Send_Check_A10_Addr 0x20692
#define Data_Check_A10_Addr 0x20694
//A10 左右桥臂载波计数归零
#define L_Carrier_A10_Addr  0x20696
#define R_Carrier_A10_Addr  0x20698

//节点11
//A11 PWM
#define Modulation_A11_Addr  0x206C2
#define Carrier_A11_Addr     0x206C4
#define Direction_A11_Addr   0x206C6
#define InialCnt_A11_Addr    0x206C8
//A11 AD
#define AD_A11_DC_Addr  0x206CA   //向该地址写入任何数据，表示让A11单元传回DC电压。读该地址数据为读取A11的DC电压
#define AD_A11_BV_Addr  0x206CC
#define AD_A11_BC_Addr  0x206CE
//A11 STATE
#define State_A11_Addr  0x206D0 //向该地址写入任何数据，表示让A11单元传回故障信息。读该地址数据为读取A11的故障信息
//A11 DA
#define DA_A11_Addr  0x206D2     //向该地址写入数据会启动A11的DA
//数据收发测试
#define Send_Check_A11_Addr 0x206D2
#define Data_Check_A11_Addr 0x206D4
//A1 左右桥臂载波计数归零
#define L_Carrier_A11_Addr  0x206D6
#define R_Carrier_A11_Addr  0x206D8

//节点12
//A12 PWM
#define Modulation_A12_Addr  0x20702
#define Carrier_A12_Addr     0x20704
#define Direction_A12_Addr   0x20706
#define InialCnt_A12_Addr    0x20708
//A12 AD
#define AD_A12_DC_Addr  0x2070A   //向该地址写入任何数据，表示让A12单元传回DC电压。读该地址数据为读取A12的DC电压
#define AD_A12_BV_Addr  0x2070C
#define AD_A12_BC_Addr  0x2070E
//A12 STATE
#define State_A12_Addr  0x20710 //向该地址写入任何数据，表示让A12单元传回故障信息。读该地址数据为读取A12的故障信息
//A12 DA
#define DA_A12_Addr  0x20712     //向该地址写入数据会启动A12的DA
//数据收发测试
#define Send_Check_A12_Addr 0x20712
#define Data_Check_A12_Addr 0x20714
//A12 左右桥臂载波计数归零
#define L_Carrier_A12_Addr  0x20716
#define R_Carrier_A12_Addr  0x20718

//-------------------------A相命令结束-------------------------------//

//-------------------------B相命令------------------------------------//
//节点1
//B1 PWM
#define Modulation_B1_Addr  0x20842
#define Carrier_B1_Addr     0x20844
#define Direction_B1_Addr   0x20846
#define InialCnt_B1_Addr    0x20848
//B1 AD
#define AD_B1_DC_Addr  0x2084A   //向该地址写入任何数据，表示让B1单元传回DC电压。读该地址数据为读取B1的DC电压
#define AD_B1_BV_Addr  0x2084C
#define AD_B1_BC_Addr  0x2084E
//B1 STATE
#define State_B1_Addr  0x20850 //向该地址写入任何数据，表示让B1单元传回故障信息。读该地址数据为读取B1的故障信息
//数据收发测试
#define Send_Check_B1_Addr  0x20852
#define Data_Check_B1_Addr  0x20854
//B1 左右桥臂载波计数归零
#define L_Carrier_B1_Addr  0x20856
#define R_Carrier_B1_Addr  0x20858

//节点2
//B2 PWM
#define Modulation_B2_Addr  0x20882
#define Carrier_B2_Addr     0x20884
#define Direction_B2_Addr   0x20886
#define InialCnt_B2_Addr    0x20888
//B2 AD
#define AD_B2_DC_Addr  0x2088A   //向该地址写入任何数据，表示让B2单元传回DC电压。读该地址数据为读取B2的DC电压
#define AD_B2_BV_Addr  0x2088C
#define AD_B2_BC_Addr  0x2088E
//B2 STATE
#define State_B2_Addr  0x20890 //向该地址写入任何数据，表示让B2单元传回故障信息。读该地址数据为读取B2的故障信息
//数据收发测试
#define Send_Check_B2_Addr  0x20892
#define Data_Check_B2_Addr  0x20894
//B2 左右桥臂载波计数归零
#define L_Carrier_B2_Addr  0x20896
#define R_Carrier_B2_Addr  0x20898

//节点3
//B3 PWM
#define Modulation_B3_Addr  0x208C2
#define Carrier_B3_Addr     0x208C4
#define Direction_B3_Addr   0x208C6
#define InialCnt_B3_Addr    0x208C8
//B3 AD
#define AD_B3_DC_Addr  0x208CA   //向该地址写入任何数据，表示让B3单元传回DC电压。读该地址数据为读取B3的DC电压
#define AD_B3_BV_Addr  0x208CC
#define AD_B3_BC_Addr  0x208CE
//B3 STATE
#define State_B3_Addr  0x208D0 //向该地址写入任何数据，表示让B3单元传回故障信息。读该地址数据为读取B3的故障信息
//数据收发测试
#define Send_Check_B3_Addr  0x208D2
#define Data_Check_B3_Addr  0x208D4
//B3 左右桥臂载波计数归零
#define L_Carrier_B3_Addr  0x208D6
#define R_Carrier_B3_Addr  0x208D8

//节点4
//B4 PWM
#define Modulation_B4_Addr  0x20902
#define Carrier_B4_Addr     0x20904
#define Direction_B4_Addr   0x20906
#define InialCnt_B4_Addr    0x20908
//B4 AD
#define AD_B4_DC_Addr  0x2090A   //向该地址写入任何数据，表示让B4单元传回DC电压。读该地址数据为读取B4的DC电压
#define AD_B4_BV_Addr  0x2090C
#define AD_B4_BC_Addr  0x2090E
//B4 STATE
#define State_B4_Addr  0x20910 //向该地址写入任何数据，表示让B4单元传回故障信息。读该地址数据为读取B4的故障信息
//数据收发测试
#define Send_Check_B4_Addr  0x20912
#define Data_Check_B4_Addr  0x20914
//B4 左右桥臂载波计数归零
#define L_Carrier_B4_Addr  0x20916
#define R_Carrier_B4_Addr  0x20918

//节点5
//B5 PWM
#define Modulation_B5_Addr  0x20942
#define Carrier_B5_Addr     0x20944
#define Direction_B5_Addr   0x20946
#define InialCnt_B5_Addr    0x20948
//B5 AD
#define AD_B5_DC_Addr  0x2094A   //向该地址写入任何数据，表示让B5单元传回DC电压。读该地址数据为读取B5的DC电压
#define AD_B5_BV_Addr  0x2094C
#define AD_B5_BC_Addr  0x2094E
//B5 STATE
#define State_B5_Addr  0x20950 //向该地址写入任何数据，表示让B5单元传回故障信息。读该地址数据为读取B5的故障信息
//数据收发测试
#define Send_Check_B5_Addr  0x20952
#define Data_Check_B5_Addr  0x20954
//B5 左右桥臂载波计数归零
#define L_Carrier_B5_Addr  0x20956
#define R_Carrier_B5_Addr  0x20958

//节点6
//B6 PWM
#define Modulation_B6_Addr  0x20982
#define Carrier_B6_Addr     0x20984
#define Direction_B6_Addr   0x20986
#define InialCnt_B6_Addr    0x20988
//B6 AD
#define AD_B6_DC_Addr  0x2098A   //向该地址写入任何数据，表示让B6单元传回DC电压。读该地址数据为读取B6的DC电压
#define AD_B6_BV_Addr  0x2098C
#define AD_B6_BC_Addr  0x2098E
//B6 STATE
#define State_B6_Addr  0x20990 //向该地址写入任何数据，表示让B6单元传回故障信息。读该地址数据为读取B6的故障信息
//数据收发测试
#define Send_Check_B6_Addr  0x20992
#define Data_Check_B6_Addr  0x20994
//B6 左右桥臂载波计数归零
#define L_Carrier_B6_Addr  0x20996
#define R_Carrier_B6_Addr  0x20998

//节点7
//B7 PWM
#define Modulation_B7_Addr  0x209C2
#define Carrier_B7_Addr     0x209C4
#define Direction_B7_Addr   0x209C6
#define InialCnt_B7_Addr    0x209C8
//B7 AD
#define AD_B7_DC_Addr  0x209CA   //向该地址写入任何数据，表示让B7单元传回DC电压。读该地址数据为读取B7的DC电压
#define AD_B7_BV_Addr  0x209CC
#define AD_B7_BC_Addr  0x209CE
//B7 STATE
#define State_B7_Addr  0x209D0 //向该地址写入任何数据，表示让B7单元传回故障信息。读该地址数据为读取B7的故障信息
//B7 DA
#define DA_B7_Addr  0x209D2     //向该地址写入数据会启动B7的DA
//数据收发测试
#define Send_Check_B7_Addr  0x209D2
#define Data_Check_B7_Addr  0x209D4
//B7 左右桥臂载波计数归零
#define L_Carrier_B7_Addr  0x209D6
#define R_Carrier_B7_Addr  0x209D8

//节点8
//B8 PWM
#define Modulation_B8_Addr  0x20A02
#define Carrier_B8_Addr     0x20A04
#define Direction_B8_Addr   0x20A06
#define InialCnt_B8_Addr    0x20A08
//B8 AD
#define AD_B8_DC_Addr  0x20A0A   //向该地址写入任何数据，表示让B8单元传回DC电压。读该地址数据为读取B8的DC电压
#define AD_B8_BV_Addr  0x20A0C
#define AD_B8_BC_Addr  0x20A0E
//B8 STATE
#define State_B8_Addr  0x20A10 //向该地址写入任何数据，表示让B8单元传回故障信息。读该地址数据为读取B8的故障信息
//B8 DA
#define DA_B8_Addr  0x20A12     //向该地址写入数据会启动B8的DA
//数据收发测试
#define Send_Check_B8_Addr  0x20A12
#define Data_Check_B8_Addr  0x20A14
//B8 左右桥臂载波计数归零
#define L_Carrier_B8_Addr  0x20A16
#define R_Carrier_B8_Addr  0x20A18

//节点9
//B9 PWM
#define Modulation_B9_Addr  0x20A42
#define Carrier_B9_Addr     0x20A44
#define Direction_B9_Addr   0x20A46
#define InialCnt_B9_Addr    0x20A48
//B9 AD
#define AD_B9_DC_Addr  0x20A4A   //向该地址写入任何数据，表示让B9单元传回DC电压。读该地址数据为读取B9的DC电压
#define AD_B9_BV_Addr  0x20A4C
#define AD_B9_BC_Addr  0x20A4E
//B9 STATE
#define State_B9_Addr  0x20A50 //向该地址写入任何数据，表示让B9单元传回故障信息。读该地址数据为读取B9的故障信息
//B9 DA
#define DA_B9_Addr  0x20A52     //向该地址写入数据会启动B9的DA
//数据收发测试
#define Send_Check_B9_Addr  0x20A52
#define Data_Check_B9_Addr  0x20A54
//B9 左右桥臂载波计数归零
#define L_Carrier_B9_Addr  0x20A56
#define R_Carrier_B9_Addr  0x20A58

//节点10
//B10 PWM
#define Modulation_B10_Addr  0x20A82
#define Carrier_B10_Addr     0x20A84
#define Direction_B10_Addr   0x20A86
#define InialCnt_B10_Addr    0x20A88
//B10 AD
#define AD_B10_DC_Addr  0x20A8A   //向该地址写入任何数据，表示让B10单元传回DC电压。读该地址数据为读取B10的DC电压
#define AD_B10_BV_Addr  0x20A8C
#define AD_B10_BC_Addr  0x20A8E
//B10 STATE
#define State_B10_Addr  0x20A90 //向该地址写入任何数据，表示让B10单元传回故障信息。读该地址数据为读取B10的故障信息
//B10 DA
#define DA_B10_Addr  0x20A92     //向该地址写入数据会启动B10的DA
//数据收发测试
#define Send_Check_B10_Addr  0x20A92
#define Data_Check_B10_Addr  0x20A94
//B10 左右桥臂载波计数归零
#define L_Carrier_B10_Addr  0x20A96
#define R_Carrier_B10_Addr  0x20A98

//节点11
//B11 PWM
#define Modulation_B11_Addr  0x20AC2
#define Carrier_B11_Addr     0x20AC4
#define Direction_B11_Addr   0x20AC6
#define InialCnt_B11_Addr    0x20AC8
//B11 AD
#define AD_B11_DC_Addr  0x20ACA   //向该地址写入任何数据，表示让B11单元传回DC电压。读该地址数据为读取B11的DC电压
#define AD_B11_BV_Addr  0x20ACC
#define AD_B11_BC_Addr  0x20ACE
//B11 STATE
#define State_B11_Addr  0x20AD0 //向该地址写入任何数据，表示让B11单元传回故障信息。读该地址数据为读取B11的故障信息
//B11 DA
#define DA_B11_Addr  0x20AD2     //向该地址写入数据会启动B11的DA
//数据收发测试
#define Send_Check_B11_Addr  0x20AD2
#define Data_Check_B11_Addr  0x20AD4
//B11 左右桥臂载波计数归零
#define L_Carrier_B11_Addr  0x20AD6
#define R_Carrier_B11_Addr  0x20AD8

//节点12
//B12 PWM
#define Modulation_B12_Addr  0x20B02
#define Carrier_B12_Addr     0x20B04
#define Direction_B12_Addr   0x20B06
#define InialCnt_B12_Addr    0x20B08
//B12 AD
#define AD_B12_DC_Addr  0x20B0A   //向该地址写入任何数据，表示让B12单元传回DC电压。读该地址数据为读取B12的DC电压
#define AD_B12_BV_Addr  0x20B0C
#define AD_B12_BC_Addr  0x20B0E
//B12 STATE
#define State_B12_Addr  0x20B10 //向该地址写入任何数据，表示让B12单元传回故障信息。读该地址数据为读取B12的故障信息
//B12 DA
#define DA_B12_Addr  0x20B12     //向该地址写入数据会启动B12的DA
//数据收发测试
#define Send_Check_B12_Addr  0x20B12
#define Data_Check_B12_Addr  0x20B14
//B12 左右桥臂载波计数归零
#define L_Carrier_B12_Addr  0x20B16
#define R_Carrier_B12_Addr  0x20B18
//----------------------B相命令结束------------------------------//



//-----------------------C相命令--------------------------------//
//节点1
//C1 PWM
#define Modulation_C1_Addr  0x20C42
#define Carrier_C1_Addr     0x20C44
#define Direction_C1_Addr   0x20C46
#define InialCnt_C1_Addr    0x20C48
//C1 AD
#define AD_C1_DC_Addr  0x20C4A   //向该地址写入任何数据，表示让C1单元传回DC电压。读该地址数据为读取C1的DC电压
#define AD_C1_BV_Addr  0x20C4C
#define AD_C1_BC_Addr  0x20C4E
//C1 STATE
#define State_C1_Addr  0x20C50 //向该地址写入任何数据，表示让C1单元传回故障信息。读该地址数据为读取C1的故障信息
//数据收发测试
#define Send_Check_C1_Addr  0x20C52
#define Data_Check_C1_Addr 0x20C54
//C1 左右桥臂载波计数归零
#define L_Carrier_C1_Addr  0x20C56
#define R_Carrier_C1_Addr  0x20C58

//节点2
//C2 PWM
#define Modulation_C2_Addr  0x20C82
#define Carrier_C2_Addr     0x20C84
#define Direction_C2_Addr   0x20C86
#define InialCnt_C2_Addr    0x20C88
//C2 AD
#define AD_C2_DC_Addr  0x20C8A   //向该地址写入任何数据，表示让C2单元传回DC电压。读该地址数据为读取C2的DC电压
#define AD_C2_BV_Addr  0x20C8C
#define AD_C2_BC_Addr  0x20C8E
//C2 STATE
#define State_C2_Addr  0x20C90 //向该地址写入任何数据，表示让C2单元传回故障信息。读该地址数据为读取C2的故障信息
//数据收发测试
#define Send_Check_C2_Addr  0x20C92
#define Data_Check_C2_Addr  0x20C94
//C2 左右桥臂载波计数归零
#define L_Carrier_C2_Addr  0x20C96
#define R_Carrier_C2_Addr  0x20C98

//节点3
//C3 PWM
#define Modulation_C3_Addr  0x20CC2
#define Carrier_C3_Addr     0x20CC4
#define Direction_C3_Addr   0x20CC6
#define InialCnt_C3_Addr    0x20CC8
//C3 AD
#define AD_C3_DC_Addr  0x20CCA   //向该地址写入任何数据，表示让C3单元传回DC电压。读该地址数据为读取C3的DC电压
#define AD_C3_BV_Addr  0x20CCC
#define AD_C3_BC_Addr  0x20CCE
//C3 STATE
#define State_C3_Addr  0x20CD0 //向该地址写入任何数据，表示让C3单元传回故障信息。读该地址数据为读取C3的故障信息
//数据收发测试
#define Send_Check_C3_Addr  0x20CD2
#define Data_Check_C3_Addr  0x20CD4
//C3 左右桥臂载波计数归零
#define L_Carrier_C3_Addr  0x20CD6
#define R_Carrier_C3_Addr  0x20CD8

//节点4
//C4 PWM
#define Modulation_C4_Addr  0x20D02
#define Carrier_C4_Addr     0x20D04
#define Direction_C4_Addr   0x20D06
#define InialCnt_C4_Addr    0x20D08
//C4 AD
#define AD_C4_DC_Addr  0x20D0A   //向该地址写入任何数据，表示让C4单元传回DC电压。读该地址数据为读取C4的DC电压
#define AD_C4_BV_Addr  0x20D0C
#define AD_C4_BC_Addr  0x20D0E
//C4 STATE
#define State_C4_Addr  0x20D10 //向该地址写入任何数据，表示让C4单元传回故障信息。读该地址数据为读取C4的故障信息
//数据收发测试
#define Send_Check_C4_Addr  0x20D12
#define Data_Check_C4_Addr  0x20D14
//C4 左右桥臂载波计数归零
#define L_Carrier_C4_Addr  0x20D16
#define R_Carrier_C4_Addr  0x20D18

//节点5
//C5 PWM
#define Modulation_C5_Addr  0x20D42
#define Carrier_C5_Addr     0x20D44
#define Direction_C5_Addr   0x20D46
#define InialCnt_C5_Addr    0x20D48
//C5 AD
#define AD_C5_DC_Addr  0x20D4A   //向该地址写入任何数据，表示让C5单元传回DC电压。读该地址数据为读取C5的DC电压
#define AD_C5_BV_Addr  0x20D4C
#define AD_C5_BC_Addr  0x20D4E
//C5 STATE
#define State_C5_Addr  0x20D50 //向该地址写入任何数据，表示让C5单元传回故障信息。读该地址数据为读取C5的故障信息
//数据收发测试
#define Send_Check_C5_Addr  0x20D52
#define Data_Check_C5_Addr  0x20D54
//C5 左右桥臂载波计数归零
#define L_Carrier_C5_Addr  0x20D56
#define R_Carrier_C5_Addr  0x20D58

//节点6
//C6 PWM
#define Modulation_C6_Addr  0x20D82
#define Carrier_C6_Addr     0x20D84
#define Direction_C6_Addr   0x20D86
#define InialCnt_C6_Addr    0x20D88
//C6 AD
#define AD_C6_DC_Addr  0x20D8A   //向该地址写入任何数据，表示让C6单元传回DC电压。读该地址数据为读取C6的DC电压
#define AD_C6_BV_Addr  0x20D8C
#define AD_C6_BC_Addr  0x20D8E
//C6 STATE
#define State_C6_Addr  0x20D90 //向该地址写入任何数据，表示让C6单元传回故障信息。读该地址数据为读取C6的故障信息
//数据收发测试
#define Send_Check_C6_Addr  0x20D92
#define Data_Check_C6_Addr  0x20D94
//C6 左右桥臂载波计数归零
#define L_Carrier_C6_Addr  0x20D96
#define R_Carrier_C6_Addr  0x20D98

//节点7
//C7 PWM
#define Modulation_C7_Addr  0x20DC2
#define Carrier_C7_Addr     0x20DC4
#define Direction_C7_Addr   0x20DC6
#define InialCnt_C7_Addr    0x20DC8
//C7 AD
#define AD_C7_DC_Addr  0x20DCA   //向该地址写入任何数荩硎救肅7单元传回DC电压。读该地址数据为读取C7的DC电压
#define AD_C7_BV_Addr  0x20DCC
#define AD_C7_BC_Addr  0x20DCE
//C7 STATE
#define State_C7_Addr  0x20DD0 //向该地址写入任何数据，表示让C7单元传回故障信息。读该地址数据为读取C7的故障信息
//C7 DA
#define DA_C7_Addr  0x20DD2     //向该地址写入数据会启动C7的DA
//数据收发测试
#define Send_Check_C7_Addr  0x20DD2
#define Data_Check_C7_Addr  0x20DD4
//C7 左右桥臂载波计数归零
#define L_Carrier_C7_Addr  0x20DD6
#define R_Carrier_C7_Addr  0x20DD8

//节点8
//C8 PWM
#define Modulation_C8_Addr  0x20E02
#define Carrier_C8_Addr     0x20E04
#define Direction_C8_Addr   0x20E06
#define InialCnt_C8_Addr    0x20E08
//C8 AD
#define AD_C8_DC_Addr  0x20E0A   //向该地址写入任何数据，表示让C8单元传回DC电压。读该地址数据为读取C8的DC电压
#define AD_C8_BV_Addr  0x20E0C
#define AD_C8_BC_Addr  0x20E0E
//C8 STATE
#define State_C8_Addr  0x20E10 //向该地址写入任何数据，表示让C8单元传回故障信息。读该地址数据为读取C8的故障信息
//C8 DA
#define DA_C8_Addr  0x20E12     //向该地址写入数据会启动C8的DA
//数据收发测试
#define Send_Check_C8_Addr  0x20E12
#define Data_Check_C8_Addr  0x20E14
//C8 左右桥臂载波计数归零
#define L_Carrier_C8_Addr  0x20E16
#define R_Carrier_C8_Addr  0x20E18

//节点9
//C9 PWM
#define Modulation_C9_Addr  0x20E42
#define Carrier_C9_Addr     0x20E44
#define Direction_C9_Addr   0x20E46
#define InialCnt_C9_Addr    0x20E48
//C9 AD
#define AD_C9_DC_Addr  0x20E4A   //向该地址写入任何数据，表示让C9单元传回DC电压。读该地址数据为读取C9的DC电压
#define AD_C9_BV_Addr  0x20E4C
#define AD_C9_BC_Addr  0x20E4E
//C9 STATE
#define State_C9_Addr  0x20E50 //向该地址写入任何数据，表示让C9单元传回故障信息。读该地址数据为读取C9的故障信息
//C9 DA
#define DA_C9_Addr  0x20E52     //向该地址写入数据会启动C9的DA
//数据收发测试
#define Send_Check_C9_Addr  0x20E52
#define Data_Check_C9_Addr  0x20E54
//C9 左右桥臂载波计数归零
#define L_Carrier_C9_Addr  0x20E56
#define R_Carrier_C9_Addr  0x20E58

//节点10
//C10 PWM
#define Modulation_C10_Addr  0x20E82
#define Carrier_C10_Addr     0x20E84
#define Direction_C10_Addr   0x20E86
#define InialCnt_C10_Addr    0x20E88
//C10 AD
#define AD_C10_DC_Addr  0x20E8A   //向该地址写入任何数荩硎救肅10单元传回DC电压。读该地址数据为读取C10的DC电压
#define AD_C10_BV_Addr  0x20E8C
#define AD_C10_BC_Addr  0x20E8E
//C10 STATE
#define State_C10_Addr  0x20E90 //向该地址写入任何数据，表示让C10单元传回故障信息。读该地址数据为读取C10的故障信息
//C10 DA
#define DA_C10_Addr  0x20E92     //向该地址写入数据会启动C10的DA
//数据收发测试
#define Send_Check_C10_Addr  0x20E92
#define Data_Check_C10_Addr  0x20E94
//C10 左右桥臂载波计数归零
#define L_Carrier_C10_Addr  0x20E96
#define R_Carrier_C10_Addr  0x20E98

//节点11
//C11 PWM
#define Modulation_C11_Addr  0x20EC2
#define Carrier_C11_Addr     0x20EC4
#define Direction_C11_Addr   0x20EC6
#define InialCnt_C11_Addr    0x20EC8
//C11 AD
#define AD_C11_DC_Addr  0x20ECA   //向该地址写入任何数据，表示让C11单元传回DC电压。读该地址数据为读取C11的DC电压
#define AD_C11_BV_Addr  0x20ECC
#define AD_C11_BC_Addr  0x20ECE
//C11 STATE
#define State_C11_Addr  0x20ED0 //向该地址写入任何数据，表示让C11单元传回故障信息。读该地址数据为读取C11的故障信息
//C11 DA
#define DA_C11_Addr  0x20ED2     //向该地址写入数据会启动C11的DA
//数据收发测试
#define Send_Check_C11_Addr  0x20ED2
#define Data_Check_C11_Addr  0x20ED4
//C11 左右桥臂载波计数归零
#define L_Carrier_C11_Addr  0x20ED6
#define R_Carrier_C11_Addr  0x20ED8

//节点12
//C12 PWM
#define Modulation_C12_Addr  0x20F02
#define Carrier_C12_Addr     0x20F04
#define Direction_C12_Addr   0x20F06
#define InialCnt_C12_Addr    0x20F08
//C12 AD
#define AD_C12_DC_Addr  0x20F0A   //向该地址写入任何数据，表示让C12单元传回DC电压。读该地址数据为读取C12的DC电压
#define AD_C12_BV_Addr  0x20F0C
#define AD_C12_BC_Addr  0x20F0E
//C12 STATE
#define State_C12_Addr  0x20F10 //向该地址写入任何数据，表示让C12单元传回故障信息。读该地址数据为读取C12的故障信息
//C12 DA
#define DA_C12_Addr  0x20F12     //向该地址写入数据会启动C12的DA
//数据收发测试
#define Send_Check_C12_Addr  0x20F12
#define Data_Check_C12_Addr  0x20F14
//C12 左右桥臂载波计数归零
#define L_Carrier_C12_Addr  0x20F16
#define R_Carrier_C12_Addr  0x20F18
//-----------------------------C相命令结束-----------------------------//

//采样数据
float  UAB_Temp[No_Sample] = {0};
float  UBC_Temp[No_Sample] = {0};
float  IA_Load_Temp[No_Sample] = {0};
float  IB_Load_Temp[No_Sample] = {0};

#endif  // end of SVG_CONSTATNT_H definition
