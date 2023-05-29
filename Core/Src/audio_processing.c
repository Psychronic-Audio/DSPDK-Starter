/*
 * audio_processing.c
 *
 *  Created on: May 12, 2023
 *      Author: Psychronic Audio
 */

#include "main.h"
#include "audio_processing.h"
#include "arm_math.h"

arm_fir_instance_f32 firSettingsBand1, firSettingsBand2, firSettingsBand3, firSettingsBand4, firSettingsBand5, firSettingsBand6;
float32_t impulse1[FILTER_IMPULSE_LENGTH] = {-0.0004146038421503351,-0.00035749882652139027,-0.0002808714415463541,-0.00018516074134661801,-7.144483818969872e-05,5.786843888199472e-05,0.00019835507774233218,0.000343028222387168,0.0004820639607657907,0.0006029110736914915,0.000690858147819712,0.0007300795750397359,0.0007051213021001717,0.000602724096974667,0.000413823644547568,0.000135520087826237,-0.00022721891384215105,-0.0006603629761007607,-0.0011403524243975393,-0.0016345341690937195,-0.0021026026860792423,-0.002499058452516358,-0.002776593989252229,-0.002890212674840468,-0.0028017878410997923,-0.0024846895719833574,-0.0019280534806199517,-0.0011402470953643115,-0.00015111025666729282,0.0009873922625430616,0.0022023668811000864,0.0034031629608300626,0.004486488884147919,0.005343270864997726,0.005866884408649942,0.005962236362225408,0.005555054319543881,0.004600662328046568,0.003091497063610789,0.0010626526758770083,-0.0014051634419808348,-0.0041857313732305695,-0.007110745287744997,-0.009976602629883539,-0.01255434165513035,-0.014602262773386245,-0.01588053912354271,-0.016166931028137942,-0.015272584169426652,-0.013056825497588376,-0.00943988215385134,-0.004412539489509535,0.0019580790740230548,0.009527194713100264,0.018075818703232067,0.027318945230558684,0.0369181507696803,0.04649794833650459,0.05566497139507851,0.06402884691077701,0.07122347685555228,0.07692739391177647,0.08088189542736508,0.08290578831565301,0.08290578831565301,0.08088189542736508,0.07692739391177647,0.07122347685555228,0.06402884691077701,0.05566497139507851,0.04649794833650459,0.0369181507696803,0.027318945230558684,0.018075818703232063,0.009527194713100264,0.0019580790740230548,-0.004412539489509535,-0.009439882153851338,-0.013056825497588376,-0.015272584169426652,-0.01616693102813794,-0.01588053912354271,-0.014602262773386245,-0.01255434165513035,-0.009976602629883535,-0.007110745287744996,-0.004185731373230568,-0.0014051634419808348,0.0010626526758770085,0.003091497063610789,0.004600662328046568,0.00555505431954388,0.005962236362225407,0.00586688440864994,0.005343270864997724,0.00448648888414792,0.0034031629608300626,0.0022023668811000864,0.0009873922625430616,-0.0001511102566672928,-0.0011402470953643111,-0.0019280534806199508,-0.0024846895719833582,-0.002801787841099793,-0.002890212674840468,-0.002776593989252229,-0.002499058452516358,-0.0021026026860792406,-0.0016345341690937184,-0.0011403524243975387,-0.0006603629761007607,-0.00022721891384215105,0.000135520087826237,0.000413823644547568,0.000602724096974667,0.0007051213021001711,0.0007300795750397354,0.0006908581478197126,0.0006029110736914915,0.0004820639607657907,0.000343028222387168,0.00019835507774233218,5.7868438881994687e-05,-7.144483818969872e-05,-0.00018516074134661801,-0.0002808714415463543,-0.00035749882652139027,-0.0004146038421503351},
							impulse2[FILTER_IMPULSE_LENGTH] = {0.0007441210557371845,0.0006680406257480019,0.0004711250279025834,0.0001739821295475541,-0.00018206951985256514,-0.000538748542504214,-0.0008293996835498981,-0.0009925205737981071,-0.0009882119764208074,-0.0008132873042016251,-0.0005094456833099444,-0.0001594343168427161,0.00013112232506885935,0.0002640274283053634,0.00018375142556547631,-9.328699803443266e-05,-0.000466984867771002,-0.0007676940248301739,-0.0007976481448131898,-0.0003921623945535042,0.0005151750030016596,0.001843760855851076,0.003352937512295224,0.004676198514388737,0.005401329345079263,0.00518046887924858,0.0038419360596148614,0.001470360038691514,-0.0015740851786940077,-0.004712598982898865,-0.007275371556389583,-0.00867238609192812,-0.008563054259684184,-0.00697456174124104,-0.004327322297604168,-0.0013480618379225332,0.0011180958888468713,0.002353678074055546,0.0020073008531986333,0.0002522893743585976,-0.0021928680970374087,-0.0042079742258934036,-0.004560053507300888,-0.002274886788450514,0.003003830316145561,0.010772001705806469,0.019624997369696747,0.02746111025487047,0.03190901686455332,0.03089726750149505,0.023236483676924168,0.009066653236658595,-0.009959850004895363,-0.030835720560585998,-0.04969558728775143,-0.06256776529364753,-0.06622738012912514,-0.0589611080384263,-0.041061295951399936,-0.014919088975536171,0.015328073102409497,0.04453895363792056,0.06755419288264054,0.08021840809437451,0.08021840809437451,0.06755419288264054,0.04453895363792056,0.015328073102409497,-0.014919088975536171,-0.041061295951399936,-0.0589611080384263,-0.06622738012912514,-0.06256776529364753,-0.049695587287751426,-0.030835720560585998,-0.009959850004895363,0.009066653236658595,0.023236483676924165,0.03089726750149505,0.03190901686455332,0.027461110254870462,0.019624997369696747,0.010772001705806469,0.003003830316145561,-0.002274886788450513,-0.004560053507300887,-0.004207974225893402,-0.0021928680970374087,0.0002522893743585976,0.0020073008531986333,0.002353678074055546,0.001118095888846871,-0.001348061837922533,-0.004327322297604166,-0.006974561741241037,-0.008563054259684186,-0.00867238609192812,-0.007275371556389583,-0.004712598982898864,-0.0015740851786940075,0.0014703600386915133,0.00384193605961486,0.005180468879248581,0.0054013293450792645,0.004676198514388737,0.003352937512295224,0.001843760855851076,0.0005151750030016591,-0.00039216239455350395,-0.0007976481448131894,-0.0007676940248301739,-0.000466984867771002,-9.328699803443266e-05,0.00018375142556547631,0.0002640274283053634,0.00013112232506885926,-0.00015943431684271598,-0.0005094456833099448,-0.0008132873042016251,-0.0009882119764208074,-0.0009925205737981071,-0.0008293996835498981,-0.0005387485425042137,-0.00018206951985256514,0.0001739821295475541,0.0004711250279025837,0.0006680406257480019,0.0007441210557371845},
							impulse3[FILTER_IMPULSE_LENGTH] = {-0.0004991235680318865,-8.881035841195222e-05,0.00015157296736189907,5.9084155572559466e-05,-0.00015271203036330348,-9.290146216640784e-05,0.00039349501845688625,0.0009765717139423047,0.0010545253398472109,0.00031902858743145397,-0.0008392104136746235,-0.0015572417575730048,-0.0012647582875027594,-0.0002840303841791267,0.00041614767244704977,0.0002047202588651234,-0.00044646532523920997,-0.0003326516846004948,0.0011543808585722352,0.0030504601542985232,0.003408786646645873,0.001171384285389358,-0.0024612902511444404,-0.00476630531711133,-0.003950826358155773,-0.0009940774272182982,0.001177968554834157,0.0006709096557053719,-0.0011936970674229948,-0.0010263794961500948,0.0028842779452978094,0.00797283611120069,0.009086389635891175,0.0034487678247269926,-0.0058648053623385115,-0.011913888481950864,-0.010117449600654069,-0.002819796690255214,0.0027625523118446995,0.001817440562244631,-0.0027038039448901044,-0.0026870358239554936,0.006280462623396442,0.0185299925225628,0.02188751450264366,0.009199222490089518,-0.013031569016216996,-0.028473589492352867,-0.02530776189961666,-0.007890000968309701,0.00661033707251945,0.0051300322886033735,-0.006642454293819112,-0.007863172277045785,0.016076097443793767,0.053542141737559244,0.06959965017529973,0.0344662817297497,-0.04486579956004218,-0.11965550595259028,-0.13122461552920786,-0.057708389159195995,0.06244958258982228,0.1527802493685939,0.1527802493685939,0.06244958258982228,-0.057708389159195995,-0.13122461552920786,-0.11965550595259028,-0.04486579956004218,0.0344662817297497,0.06959965017529973,0.053542141737559244,0.016076097443793767,-0.007863172277045785,-0.006642454293819112,0.0051300322886033735,0.006610337072519449,-0.007890000968309701,-0.02530776189961666,-0.02847358949235286,-0.013031569016216996,0.009199222490089518,0.02188751450264366,0.018529992522562794,0.006280462623396441,-0.0026870358239554928,-0.0027038039448901044,0.0018174405622446313,0.0027625523118446995,-0.002819796690255214,-0.010117449600654067,-0.01191388848195086,-0.005864805362338509,0.0034487678247269913,0.009086389635891177,0.00797283611120069,0.0028842779452978094,-0.0010263794961500945,-0.0011936970674229948,0.0006709096557053716,0.0011779685548341566,-0.0009940774272182986,-0.003950826358155774,-0.00476630531711133,-0.0024612902511444404,0.001171384285389358,0.0034087866466458706,0.003050460154298521,0.0011543808585722348,-0.0003326516846004948,-0.00044646532523920997,0.0002047202588651234,0.00041614767244704977,-0.0002840303841791267,-0.0012647582875027585,-0.0015572417575730037,-0.0008392104136746241,0.00031902858743145397,0.0010545253398472109,0.0009765717139423047,0.00039349501845688625,-9.290146216640779e-05,-0.00015271203036330348,5.9084155572559466e-05,0.00015157296736189915,-8.881035841195222e-05,-0.0004991235680318865},
							impulse4[FILTER_IMPULSE_LENGTH] = {-0.00018118256133188148,-0.0005795737680213528,-0.00011050889597107197,0.00019676610921859212,-4.2511708683308645e-06,0.00013437261165995525,0.0005458892894396537,1.0865157517890184e-05,-0.0011123013346082586,-0.0007269914172821303,0.0011140897820830912,0.0014926935522695756,-0.00039591650926819804,-0.0014911541056479014,-0.0003243201467338248,0.0005165643140525144,-6.643836588415086e-05,0.00043709883502067557,0.0018110522377578102,0.00010611053239350844,-0.003551140979825729,-0.002399736584090734,0.00337640514691519,0.004611507906104602,-0.0010559361336139996,-0.004306026447631012,-0.0009774673754484394,0.0013002660193382888,-0.0003437059171650796,0.0012432632994897942,0.005132278264765883,0.000481895945072531,-0.009354434103200885,-0.006474598872800403,0.008345667183637582,0.011637444179543249,-0.0022791584456263703,-0.010264620743331763,-0.002434845058668358,0.002716883819892492,-0.0012162880716877229,0.003029450924724679,0.012724781976448038,0.0016354575629596052,-0.022279549735530656,-0.016052887516342627,0.019325096581905784,0.028086860201351208,-0.004724171619568349,-0.02447102299057955,-0.006182137322757158,0.005889487254620683,-0.004256391621889155,0.00830299173215347,0.03727497706690738,0.0063406783309003754,-0.07008092828870621,-0.056516231724046156,0.06972373956688034,0.11922934474209762,-0.01983410585201248,-0.15411914934141693,-0.06199797605514992,0.1328285093852144,0.1328285093852144,-0.06199797605514992,-0.15411914934141693,-0.01983410585201248,0.11922934474209762,0.06972373956688034,-0.056516231724046156,-0.07008092828870621,0.0063406783309003754,0.037274977066907376,0.00830299173215347,-0.004256391621889155,0.005889487254620683,-0.006182137322757157,-0.02447102299057955,-0.004724171619568349,0.0280868602013512,0.019325096581905784,-0.016052887516342627,-0.022279549735530656,0.0016354575629596048,0.012724781976448036,0.0030294509247246783,-0.0012162880716877229,0.0027168838198924922,-0.002434845058668358,-0.010264620743331763,-0.00227915844562637,0.011637444179543247,0.00834566718363758,-0.0064745988728004,-0.009354434103200889,0.000481895945072531,0.005132278264765883,0.0012432632994897942,-0.00034370591716507954,0.0013002660193382884,-0.0009774673754484391,-0.004306026447631013,-0.001055936133614,0.004611507906104602,0.00337640514691519,-0.002399736584090734,-0.0035511409798257266,0.00010611053239350837,0.0018110522377578094,0.00043709883502067557,-6.643836588415086e-05,0.0005165643140525144,-0.0003243201467338248,-0.0014911541056479014,-0.00039591650926819777,0.0014926935522695745,0.001114089782083092,-0.0007269914172821303,-0.0011123013346082586,1.0865157517890184e-05,0.0005458892894396537,0.00013437261165995517,-4.2511708683308645e-06,0.00019676610921859212,-0.00011050889597107204,-0.0005795737680213528,-0.00018118256133188148},
							impulse5[FILTER_IMPULSE_LENGTH] = {0.0006005934984859642,-8.885238454095692e-05,-0.00024608886059331763,3.9694299105303286e-05,-0.00010192932707128637,0.0004237565803118908,5.125809227761114e-05,-0.00098209215989451,0.0005449730308414955,0.0011003777445104993,-0.0013042987331924882,-0.0005299029591676084,0.0014839534974899512,-0.00022392087321481598,-0.0007212397706221175,0.00013670223530330944,-0.00029643758607081355,0.001283413003572019,0.00018816721782376878,-0.003102643737275403,0.0016767190058042533,0.0035003375810668843,-0.004037419037390761,-0.0016958205522762885,0.0045282074149616955,-0.0006313068806977687,-0.002190388457130506,0.0004457873981835394,-0.0007895660428071313,0.0034305792918233975,0.0005780242693011355,-0.008212694930399448,0.004258358428101921,0.009123286334640036,-0.010209546280868492,-0.0044329172637308425,0.011347047034124303,-0.0014635528983375885,-0.005543215759020604,0.0012030188918495204,-0.0017840998382347396,0.007922283804605683,0.0015302768347061444,-0.01935782461686501,0.009782051425391151,0.02190131239556757,-0.024227076033380437,-0.011082800976634416,0.027819786685385265,-0.0033774237615143143,-0.01440925874017692,0.0033868089973484628,-0.004378229896200092,0.020740397931904627,0.004732235389937457,-0.0568073279029635,0.02968670515388098,0.07481562278977305,-0.08973671415490893,-0.048741169825830905,0.14160853888304106,-0.0204729209869794,-0.14904565723847066,0.10072069290182549,0.10072069290182549,-0.14904565723847066,-0.0204729209869794,0.14160853888304106,-0.048741169825830905,-0.08973671415490893,0.07481562278977305,0.02968670515388098,-0.0568073279029635,0.004732235389937456,0.020740397931904627,-0.004378229896200092,0.0033868089973484628,-0.014409258740176917,-0.0033774237615143143,0.027819786685385265,-0.011082800976634414,-0.024227076033380437,0.02190131239556757,0.009782051425391151,-0.019357824616865003,0.0015302768347061442,0.00792228380460568,-0.0017840998382347396,0.0012030188918495206,-0.005543215759020604,-0.0014635528983375885,0.011347047034124301,-0.004432917263730842,-0.010209546280868488,0.009123286334640032,0.004258358428101922,-0.008212694930399448,0.0005780242693011355,0.003430579291823397,-0.0007895660428071312,0.00044578739818353926,-0.002190388457130505,-0.0006313068806977689,0.004528207414961697,-0.0016958205522762885,-0.004037419037390761,0.0035003375810668843,0.001676719005804252,-0.003102643737275401,0.0001881672178237687,0.001283413003572019,-0.00029643758607081355,0.00013670223530330944,-0.0007212397706221175,-0.00022392087321481598,0.0014839534974899504,-0.0005299029591676079,-0.0013042987331924893,0.0011003777445104993,0.0005449730308414955,-0.00098209215989451,5.125809227761114e-05,0.00042375658031189057,-0.00010192932707128637,3.9694299105303286e-05,-0.0002460888605933178,-8.885238454095692e-05,0.0006005934984859642},
							impulse6[FILTER_IMPULSE_LENGTH] = {-0.0006697566849439562,0.0007902011063867339,-0.00042818011869057984,-0.00011650674730613474,0.0004091620435502125,-0.0002805853516456855,5.177748394699108e-06,-4.425038831399955e-05,0.0005399373532995868,-0.0010517826116158773,0.0008886162777028364,0.0001771397649541952,-0.0015084843691460517,0.002041983590219312,-0.0012836147865356838,-0.00010812025973710119,0.0008758763223045317,-0.0004550733948169255,-0.00031087744529807964,-4.035992627463544e-05,0.0018971210186001246,-0.0037419196618608866,0.0033402382047157367,-5.987842165651082e-05,-0.004029202978142325,0.005733394292934308,-0.0037654151031556358,0.00019272041396001187,0.0014909315200771177,-4.147973087604532e-05,-0.001917772111996431,0.0004563639171363475,0.005112573408852221,-0.010464453580296337,0.009785709397485019,-0.00169487597243485,-0.008436752655266005,0.012831195384331431,-0.008683562789648719,0.0011800387185470227,0.0015735854809200741,0.0027567957613225435,-0.007413637578286732,0.0030337477372165424,0.011650289633283576,-0.02597385943109224,0.025832248259412424,-0.007601259683980308,-0.01633698639965784,0.02760169182083772,-0.019240557773730446,0.0034363909712720984,3.878545670606651e-05,0.0146471579267715,-0.029277397258008566,0.016157624323395625,0.03264927166715476,-0.08861223699825317,0.10260398000610357,-0.0452831222769792,-0.06102591380609353,0.14957252515092823,-0.15532450116840354,0.0656834503109673,0.0656834503109673,-0.15532450116840354,0.14957252515092823,-0.06102591380609353,-0.0452831222769792,0.10260398000610357,-0.08861223699825317,0.03264927166715476,0.016157624323395625,-0.029277397258008563,0.0146471579267715,3.878545670606651e-05,0.0034363909712720984,-0.019240557773730443,0.02760169182083772,-0.01633698639965784,-0.007601259683980306,0.025832248259412424,-0.02597385943109224,0.011650289633283576,0.0030337477372165416,-0.007413637578286731,0.0027567957613225426,0.0015735854809200741,0.0011800387185470229,-0.008683562789648719,0.012831195384331431,-0.008436752655266003,-0.0016948759724348499,0.009785709397485015,-0.010464453580296334,0.005112573408852222,0.0004563639171363475,-0.001917772111996431,-4.1479730876045314e-05,0.0014909315200771177,0.0001927204139600118,-0.003765415103155634,0.00573339429293431,-0.004029202978142327,-5.987842165651082e-05,0.0033402382047157367,-0.0037419196618608866,0.0018971210186001233,-4.0359926274635414e-05,-0.0003108774452980795,-0.0004550733948169255,0.0008758763223045317,-0.00010812025973710119,-0.0012836147865356838,0.002041983590219312,-0.0015084843691460506,0.00017713976495419505,0.000888616277702837,-0.0010517826116158773,0.0005399373532995868,-4.425038831399955e-05,5.177748394699108e-06,-0.00028058535164568536,0.0004091620435502125,-0.00011650674730613474,-0.00042818011869058016,0.0007902011063867339,-0.0006697566849439562};

float32_t fir_state_1[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_2[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_3[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_4[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_5[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_6[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1];

float32_t fir_out_1[BUFFER_SIZE/2],
	fir_out_2[BUFFER_SIZE/2],
	fir_out_3[BUFFER_SIZE/2],
	fir_out_4[BUFFER_SIZE/2],
	fir_out_5[BUFFER_SIZE/2],
	fir_out_6[BUFFER_SIZE/2];

void initialize_filters(){
	arm_fir_init_f32(&firSettingsBand1, FILTER_IMPULSE_LENGTH, &impulse1[0], &fir_state_1[0], BUFFER_SIZE/2);
	arm_fir_init_f32(&firSettingsBand2, FILTER_IMPULSE_LENGTH, &impulse2[0], &fir_state_2[0], BUFFER_SIZE/2);
	arm_fir_init_f32(&firSettingsBand3, FILTER_IMPULSE_LENGTH, &impulse3[0], &fir_state_3[0], BUFFER_SIZE/2);
	arm_fir_init_f32(&firSettingsBand4, FILTER_IMPULSE_LENGTH, &impulse4[0], &fir_state_4[0], BUFFER_SIZE/2);
	arm_fir_init_f32(&firSettingsBand5, FILTER_IMPULSE_LENGTH, &impulse5[0], &fir_state_5[0], BUFFER_SIZE/2);
	arm_fir_init_f32(&firSettingsBand6, FILTER_IMPULSE_LENGTH, &impulse6[0], &fir_state_6[0], BUFFER_SIZE/2);
}

void process_audio(volatile int32_t *inputSamples, volatile int32_t *outputSamples){
	int32_t leftChannel[BUFFER_SIZE/2];
	float leftChannelFloat[BUFFER_SIZE/2];

	//The data collected from the ADC and DAC are still stereo. We need to separate the left channel out of the mix
	for(int i = 0; i < BUFFER_SIZE; i += 2){
		int index = 0;
		leftChannel[index] = inputSamples[i];
		index += 1;
	}

	//The data collected is in 24-bit signed format on a 32 bit frame, to do DSP we want the data to be in float format from [-1.0, +1.0]
	for(int i = 0; i < BUFFER_SIZE/2; i++){
		leftChannelFloat[i] = INT24_TO_FLOAT * leftChannel[i];
		if(leftChannelFloat[i] > 1.0f){
			leftChannelFloat[i] -= 2.0f;
		}
	}

	if(EQ_Enable == TRUE){

		//AUDIO PROCESSING (THIS IS WHERE THE DSP HAPPENS)

		arm_fir_f32 (&firSettingsBand1, leftChannelFloat, fir_out_1, BUFFER_SIZE/2);
		arm_fir_f32 (&firSettingsBand1, leftChannelFloat, fir_out_2, BUFFER_SIZE/2);
		arm_fir_f32 (&firSettingsBand1, leftChannelFloat, fir_out_3, BUFFER_SIZE/2);
		arm_fir_f32 (&firSettingsBand1, leftChannelFloat, fir_out_4, BUFFER_SIZE/2);
		arm_fir_f32 (&firSettingsBand1, leftChannelFloat, fir_out_5, BUFFER_SIZE/2);
		arm_fir_f32 (&firSettingsBand1, leftChannelFloat, fir_out_6, BUFFER_SIZE/2);

		//END AUDIO PROCESSING

		//Convert back from float to signed 24-bit integer, and place processed signal in DAC's buffer

		for(int i = 0; i < BUFFER_SIZE; i += 2){
			outputSamples[i] = (int32_t)(FLOAT_TO_INT24 * ((fir_out_1[i] * gains[0])
								+ (fir_out_2[i] * gains[1])
								+ (fir_out_3[i] * gains[2])
								+ (fir_out_4[i] * gains[3])
								+ (fir_out_5[i] * gains[4])
								+ (fir_out_6[i] * gains[5])) * gains[6]);
			outputSamples[i+1] = 0;
		}
	}else{
		//Audio passthrough if not enabled
		for(int i = 0; i < BUFFER_SIZE; i += 2){
				outputSamples[i] = (int32_t)(FLOAT_TO_INT24 * leftChannelFloat[i/2]);
				outputSamples[i+1] = 0;
		}
	}

}
