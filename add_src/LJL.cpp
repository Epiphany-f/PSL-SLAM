#include "LJL.h"

namespace ORB_SLAM2 {
	const float HARRIS_K = 0.04f;
	static int bit_pattern_31_[256*4] =
	{
		8,-3, 9,5/*mean (0), correlation (0)*/,
		4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
		-11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
		7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
		2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
		1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
		-2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
		-13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
		-13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
		10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
		-13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
		-11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
		7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
		-4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
		-13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
		-9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
		12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
		-3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
		-6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
		11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
		4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
		5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
		3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
		-8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
		-2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
		-13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
		-7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
		-4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
		-10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
		5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
		5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
		1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
		9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
		4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
		2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
		-4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
		-8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
		4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
		0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
		-13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
		-3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
		-6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
		8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
		0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
		7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
		-13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
		10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
		-6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
		10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
		-13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
		-13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
		3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
		5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
		-1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
		3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
		2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
		-13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
		-13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
		-13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
		-7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
		6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
		-9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
		-2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
		-12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
		3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
		-7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
		-3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
		2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
		-11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
		-1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
		5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
		-4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
		-9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
		-12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
		10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
		7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
		-7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
		-4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
		7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
		-7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
		-13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
		-3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
		7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
		-13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
		1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
		2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
		-4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
		-1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
		7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
		1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
		9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
		-1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
		-13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
		7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
		12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
		6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
		5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
		2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
		3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
		2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
		9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
		-8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
		-11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
		1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
		6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
		2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
		6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
		3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
		7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
		-11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
		-10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
		-5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
		-10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
		8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
		4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
		-10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
		4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
		-2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
		-5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
		7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
		-9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
		-5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
		8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
		-9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
		1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
		7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
		-2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
		11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
		-12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
		3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
		5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
		0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
		-9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
		0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
		-1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
		5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
		3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
		-13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
		-5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
		-4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
		6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
		-7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
		-13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
		1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
		4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
		-2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
		2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
		-2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
		4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
		-6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
		-3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
		7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
		4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
		-13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
		7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
		7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
		-7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
		-8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
		-13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
		2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
		10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
		-6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
		8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
		2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
		-11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
		-12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
		-11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
		5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
		-2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
		-1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
		-13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
		-10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
		-3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
		2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
		-9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
		-4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
		-4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
		-6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
		6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
		-13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
		11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
		7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
		-1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
		-4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
		-7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
		-13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
		-7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
		-8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
		-5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
		-13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
		1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
		1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
		9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
		5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
		-1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
		-9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
		-1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
		-13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
		8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
		2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
		7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
		-10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
		-10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
		4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
		3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
		-4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
		5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
		4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
		-9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
		0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
		-12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
		3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
		-10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
		8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
		-8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
		2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
		10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
		6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
		-7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
		-3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
		-1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
		-3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
		-8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
		4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
		2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
		6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
		3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
		11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
		-3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
		4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
		2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
		-10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
		-13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
		-13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
		6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
		0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
		-13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
		-9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
		-13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
		5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
		2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
		-1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
		9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
		11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
		3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
		-1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
		3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
		-13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
		5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
		8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
		7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
		-10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
		7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
		9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
		7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
		-1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
	};

	static void makeRandomPattern(int patchSize, Point* pattern, int npoints)
	{
		RNG rng(0x34985739); // we always start with a fixed seed,
		// to make patterns the same on each run
		for( int i = 0; i < npoints; i++ )
		{
			pattern[i].x = rng.uniform(-patchSize/2, patchSize/2+1);
			pattern[i].y = rng.uniform(-patchSize/2, patchSize/2+1);
		}
	}

	static void
	computeOrbDescriptors( const Mat& imagePyramid, const std::vector<Rect>& layerInfo,
						   const std::vector<float>& layerScale, std::vector<KeyPoint>& keypoints,
						   Mat& descriptors, const std::vector<Point>& _pattern, int dsize, int wta_k )
	{
		int step = (int)imagePyramid.step;
		int j, i, nkeypoints = (int)keypoints.size();

		for( j = 0; j < nkeypoints; j++ )
		{
			const KeyPoint& kpt = keypoints[j];
			const Rect& layer = layerInfo[kpt.octave];
			float scale = 1.f/layerScale[kpt.octave];
			float angle = kpt.angle;

			angle *= (float)(CV_PI/180.f);
			float a = (float)cos(angle), b = (float)sin(angle);

			const uchar* center = &imagePyramid.at<uchar>(cvRound(kpt.pt.y*scale) + layer.y,
														  cvRound(kpt.pt.x*scale) + layer.x);
			float x, y;
			int ix, iy;
			const Point* pattern = &_pattern[0];
			uchar* desc = descriptors.ptr<uchar>(j);

#if 1
#define GET_VALUE(idx) \
(x = pattern[idx].x*a - pattern[idx].y*b, \
y = pattern[idx].x*b + pattern[idx].y*a, \
ix = cvRound(x), \
iy = cvRound(y), \
*(center + iy*step + ix) )
#else
#define GET_VALUE(idx) \
(x = pattern[idx].x*a - pattern[idx].y*b, \
y = pattern[idx].x*b + pattern[idx].y*a, \
ix = cvFloor(x), iy = cvFloor(y), \
x -= ix, y -= iy, \
cvRound(center[iy*step + ix]*(1-x)*(1-y) + center[(iy+1)*step + ix]*(1-x)*y + \
center[iy*step + ix+1]*x*(1-y) + center[(iy+1)*step + ix+1]*x*y))
#endif

			if( wta_k == 2 )
			{
				for (i = 0; i < dsize; ++i, pattern += 16)
				{
					int t0, t1, val;
					t0 = GET_VALUE(0); t1 = GET_VALUE(1);
					val = t0 < t1;
					t0 = GET_VALUE(2); t1 = GET_VALUE(3);
					val |= (t0 < t1) << 1;
					t0 = GET_VALUE(4); t1 = GET_VALUE(5);
					val |= (t0 < t1) << 2;
					t0 = GET_VALUE(6); t1 = GET_VALUE(7);
					val |= (t0 < t1) << 3;
					t0 = GET_VALUE(8); t1 = GET_VALUE(9);
					val |= (t0 < t1) << 4;
					t0 = GET_VALUE(10); t1 = GET_VALUE(11);
					val |= (t0 < t1) << 5;
					t0 = GET_VALUE(12); t1 = GET_VALUE(13);
					val |= (t0 < t1) << 6;
					t0 = GET_VALUE(14); t1 = GET_VALUE(15);
					val |= (t0 < t1) << 7;

					desc[i] = (uchar)val;
				}
			}
			else if( wta_k == 3 )
			{
				for (i = 0; i < dsize; ++i, pattern += 12)
				{
					int t0, t1, t2, val;
					t0 = GET_VALUE(0); t1 = GET_VALUE(1); t2 = GET_VALUE(2);
					val = t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0);

					t0 = GET_VALUE(3); t1 = GET_VALUE(4); t2 = GET_VALUE(5);
					val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 2;

					t0 = GET_VALUE(6); t1 = GET_VALUE(7); t2 = GET_VALUE(8);
					val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 4;

					t0 = GET_VALUE(9); t1 = GET_VALUE(10); t2 = GET_VALUE(11);
					val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 6;

					desc[i] = (uchar)val;
				}
			}
			else if( wta_k == 4 )
			{
				for (i = 0; i < dsize; ++i, pattern += 16)
				{
					int t0, t1, t2, t3, u, v, k, val;
					t0 = GET_VALUE(0); t1 = GET_VALUE(1);
					t2 = GET_VALUE(2); t3 = GET_VALUE(3);
					u = 0, v = 2;
					if( t1 > t0 ) t0 = t1, u = 1;
					if( t3 > t2 ) t2 = t3, v = 3;
					k = t0 > t2 ? u : v;
					val = k;

					t0 = GET_VALUE(4); t1 = GET_VALUE(5);
					t2 = GET_VALUE(6); t3 = GET_VALUE(7);
					u = 0, v = 2;
					if( t1 > t0 ) t0 = t1, u = 1;
					if( t3 > t2 ) t2 = t3, v = 3;
					k = t0 > t2 ? u : v;
					val |= k << 2;

					t0 = GET_VALUE(8); t1 = GET_VALUE(9);
					t2 = GET_VALUE(10); t3 = GET_VALUE(11);
					u = 0, v = 2;
					if( t1 > t0 ) t0 = t1, u = 1;
					if( t3 > t2 ) t2 = t3, v = 3;
					k = t0 > t2 ? u : v;
					val |= k << 4;

					t0 = GET_VALUE(12); t1 = GET_VALUE(13);
					t2 = GET_VALUE(14); t3 = GET_VALUE(15);
					u = 0, v = 2;
					if( t1 > t0 ) t0 = t1, u = 1;
					if( t3 > t2 ) t2 = t3, v = 3;
					k = t0 > t2 ? u : v;
					val |= k << 6;

					desc[i] = (uchar)val;
				}
			}
			else
				CV_Error( Error::StsBadSize, "Wrong wta_k. It can be only 2, 3 or 4." );
#undef GET_VALUE
		}
	}

	static void initializeOrbPattern( const Point* pattern0, std::vector<Point>& pattern, int ntuples, int tupleSize, int poolSize )
	{
		RNG rng(0x12345678);
		int i, k, k1;
		pattern.resize(ntuples*tupleSize);

		for( i = 0; i < ntuples; i++ )
		{
			for( k = 0; k < tupleSize; k++ )
			{
				for(;;)
				{
					int idx = rng.uniform(0, poolSize);
					Point pt = pattern0[idx];
					for( k1 = 0; k1 < k; k1++ )
						if( pattern[tupleSize*i + k1] == pt )
							break;
					if( k1 == k )
					{
						pattern[tupleSize*i + k] = pt;
						break;
					}
				}
			}
		}
	}

	static void ICAngles(const Mat& img, const std::vector<Rect>& layerinfo,
						 std::vector<KeyPoint>& pts, const std::vector<int> & u_max, int half_k)
	{
		int step = (int)img.step1();
		size_t ptidx, ptsize = pts.size();


		for( ptidx = 0; ptidx < ptsize; ptidx++ )
		{
			const Rect& layer = layerinfo[pts[ptidx].octave];
			const uchar* center = &img.at<uchar>(cvRound(pts[ptidx].pt.y) + layer.y, cvRound(pts[ptidx].pt.x) + layer.x);

			int m_01 = 0, m_10 = 0;

			// Treat the center line differently, v=0
			for (int u = -half_k; u <= half_k; ++u)
				m_10 += u * center[u];

			// Go line by line in the circular patch
			for (int v = 1; v <= half_k; ++v)
			{
				// Proceed over the two lines
				int v_sum = 0;
				int d = u_max[v];
				for (int u = -d; u <= d; ++u)
				{
					int val_plus = center[u + v*step], val_minus = center[u - v*step];
					v_sum += (val_plus - val_minus);
					m_10 += u * (val_plus + val_minus);
				}
				m_01 += v * v_sum;
			}

			pts[ptidx].angle = fastAtan2((float)m_01, (float)m_10);
		}
	}

	static void
	HarrisResponses(const Mat& img, const std::vector<Rect>& layerinfo,
					std::vector<KeyPoint>& pts, int blockSize, float harris_k)
	{
		CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );

		size_t ptidx, ptsize = pts.size();

		const uchar* ptr00 = img.ptr<uchar>();
		int step = (int)(img.step/img.elemSize1());
		int r = blockSize/2;

		float scale = 1.f/((1 << 2) * blockSize * 255.f);
		float scale_sq_sq = scale * scale * scale * scale;

		AutoBuffer<int> ofsbuf(blockSize*blockSize);
		int* ofs = ofsbuf;
		for( int i = 0; i < blockSize; i++ )
			for( int j = 0; j < blockSize; j++ )
				ofs[i*blockSize + j] = (int)(i*step + j);

		for( ptidx = 0; ptidx < ptsize; ptidx++ )
		{
			int x0 = cvRound(pts[ptidx].pt.x);
			int y0 = cvRound(pts[ptidx].pt.y);
			int z = pts[ptidx].octave;

			const uchar* ptr0 = ptr00 + (y0 - r + layerinfo[z].y)*step + x0 - r + layerinfo[z].x;
			int a = 0, b = 0, c = 0;

			for( int k = 0; k < blockSize*blockSize; k++ )
			{
				const uchar* ptr = ptr0 + ofs[k];
				int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
				int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
				a += Ix*Ix;
				b += Iy*Iy;
				c += Ix*Iy;
			}
			pts[ptidx].response = ((float)a * b - (float)c * c -
								   harris_k * ((float)a + b) * ((float)a + b))*scale_sq_sq;
		}
	}

	void ORB_Impl_LJL::computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax, ORB_Impl_LJL::vPL temp)
	{
		// for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
		//  keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
		// {
		// 	keypoint->angle = IC_Angle(image, keypoint->pt, umax);
		// }
		for (int i = 0; i < keypoints.size(); i++)
		{
			keypoints[i].angle = temp[i].angle;
		}
	}

	void ORB_Impl_LJL::computeKeyPoints(const Mat& imagePyramid,
								 const Mat& maskPyramid,
								 const std::vector<Rect>& layerInfo,
								 const UMat& ulayerInfo,
								 const std::vector<float>& layerScale,
								 std::vector<KeyPoint>& allKeypoints,
								 vector<pair<int,pairline>>& vpairline_all,
								 int nfeatures, double scaleFactor,
								 int edgeThreshold, int patchSize, int scoreType,
								 int fastThreshold  )
	{
		int i, nkeypoints, level, nlevels = (int)layerInfo.size();
		std::vector<int> nfeaturesPerLevel(nlevels);

		// Keep more points than necessary as FAST does not give amazing corners
		float factor = (float)(1.0 / scaleFactor);
		float ndesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)std::pow((double)factor, (double)nlevels));

		int sumFeatures = 0;
		for( level = 0; level < nlevels-1; level++ )
		{
			nfeaturesPerLevel[level] = cvRound(ndesiredFeaturesPerScale);
			sumFeatures += nfeaturesPerLevel[level];
			ndesiredFeaturesPerScale *= factor;
		}
		nfeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

		// Make sure we forget about what is too close to the boundary
		//edge_threshold_ = std::max(edge_threshold_, patch_size_/2 + kKernelWidth / 2 + 2);

		// pre-compute the end of a row in a circular patch
		int halfPatchSize = patchSize / 2;
		std::vector<int> umax(halfPatchSize + 2);

		int v, v0, vmax = cvFloor(halfPatchSize * std::sqrt(2.f) / 2 + 1);
		int vmin = cvCeil(halfPatchSize * std::sqrt(2.f) / 2);
		for (v = 0; v <= vmax; ++v)
			umax[v] = cvRound(std::sqrt((double)halfPatchSize * halfPatchSize - v * v));

		// Make sure we are symmetric
		for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
		{
			while (umax[v0] == umax[v0 + 1])
				++v0;
			umax[v] = v0;
			++v0;
		}
		vector<vPL> LIL_gathers_all;
		int max_class_id = -1;
		int max_id = -1;
		allKeypoints.clear();
		vpairline_level.clear();
		vpairline_all.clear();
		vpairline_image_pyramid.clear();
		LIL_gathers.clear();
		std::vector<KeyPoint> keypoints;
		// vector<pair<int,pairline>> vpairlines_LJL;
		std::vector<int> counters(nlevels);
		keypoints.reserve(nfeaturesPerLevel[0]*2);
		vpairline_image_pyramid.resize(nlevels);
		LIL_gathers.resize(nlevels);

		for( level = 0; level < nlevels; level++ )
		{
			int featuresNum = nfeaturesPerLevel[level];
			Mat img = imagePyramid(layerInfo[level]);
			Mat mask = maskPyramid.empty() ? Mat() : maskPyramid(layerInfo[level]);

			// Detect FAST features, 20 is a good threshold
			{
				// Ptr<FastFeatureDetector> fd = FastFeatureDetector::create(fastThreshold, true);
				// fd->detect(img, keypoints, mask);

				LIL_gathers = ExactPairlinesFromJuction(img,level);
				LIL_gathers = foroctave(LIL_gathers,level,layerScale,max_id,imagePyramid,layerInfo);
				keypoints = PairlinetoVectorKeypoint(LIL_gathers,max_class_id);
				// LIL_gathers_all.push_back(LIL_gathers);

			}
			// //计算角度
			//    {
			//    	// temp = ExactPairlinesFromJuction(img,level);
			// computeOrientation(img, keypoints, umax, temp);
			//    }

			// Remove keypoints very close to the border
			KeyPointsFilter::runByImageBorder(keypoints, img.size(), edgeThreshold);

			// Keep more points than necessary as FAST does not give amazing corners
			KeyPointsFilter::retainBest(keypoints, scoreType == ORB::HARRIS_SCORE ? 2 * featuresNum : featuresNum);

			nkeypoints = (int)keypoints.size();
			counters[level] = nkeypoints;

			float sf = layerScale[level];
			for( i = 0; i < nkeypoints; i++ )
			{
				keypoints[i].octave = level;
				keypoints[i].size = patchSize*sf;
			}

			std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(allKeypoints));
			// vpairline_image_pyramid[level] = vpairline_level;
			LIL_gathers_all.push_back(LIL_gathers);
		}


		nkeypoints = (int)allKeypoints.size();
		if(nkeypoints == 0)
		{
			return;
		}

		// Select best features using the Harris cornerness (better scoring than FAST)
		if( scoreType == ORB::HARRIS_SCORE )
		{
			HarrisResponses(imagePyramid, layerInfo, allKeypoints, 7, ORB_Impl_LJL::HARRIS_K);

			std::vector<KeyPoint> newAllKeypoints;
			newAllKeypoints.reserve(nfeaturesPerLevel[0]*nlevels);

			vector<pair<int,pairline>> newPairlines_all;
			// newPairlines_all.reserve(nfeaturesPerLevel[0]*nlevels);

			int offset = 0;
			for( level = 0; level < nlevels; level++ )
			{
				int featuresNum = nfeaturesPerLevel[level];
				nkeypoints = counters[level];
				keypoints.resize(nkeypoints);
				std::copy(allKeypoints.begin() + offset,
						  allKeypoints.begin() + offset + nkeypoints,
						  keypoints.begin());
				offset += nkeypoints;

				//cull to the final desired level, using the new Harris scores.
				KeyPointsFilter::retainBest(keypoints, featuresNum);

				std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(newAllKeypoints));

				vpairline_level = GetPairline(LIL_gathers_all[level],keypoints);
				vpairline_image_pyramid[level] = vpairline_level;
				std::copy(vpairline_level.begin(), vpairline_level.end(), std::back_inserter(newPairlines_all));
			}
			std::swap(allKeypoints, newAllKeypoints);

			std::swap(vpairline_all,newPairlines_all);
		}

		nkeypoints = (int)allKeypoints.size();
		//计算角度
		{
			ICAngles(imagePyramid, layerInfo, allKeypoints, umax, halfPatchSize);
		}

		for( i = 0; i < nkeypoints; i++ )
		{
			float scale = layerScale[allKeypoints[i].octave];
			allKeypoints[i].pt *= scale;
		}
	}

	static inline float getScale(int level, int firstLevel, double scaleFactor)
	{
		return (float)std::pow(scaleFactor, (double)(level - firstLevel));
	}

	void ORB_Impl_LJL::detectAndCompute( InputArray _image, InputArray _mask,
									 std::vector<KeyPoint>& keypoints,
									 vector<pair<int,pairline>>& vpairlines_all,
									 OutputArray _descriptors, bool useProvidedKeypoints )
	{

		// CV_Assert(patchSize >= 2);

		bool do_keypoints = !useProvidedKeypoints;
		bool do_descriptors = _descriptors.needed();

		if( (!do_keypoints && !do_descriptors) || _image.empty() )
			return;

		//ROI handling
		const int HARRIS_BLOCK_SIZE = 9;
		int halfPatchSize = patchSize / 2;
		// sqrt(2.0) is for handling patch rotation
		int descPatchSize = cvCeil(halfPatchSize*sqrt(2.0));
		int border = std::max(edgeThreshold, std::max(descPatchSize, HARRIS_BLOCK_SIZE/2))+1;


		Mat image = _image.getMat(), mask = _mask.getMat();
		if( image.type() != CV_8UC1 )
			cvtColor(_image, image, COLOR_BGR2GRAY);

		int i, level, nLevels = this->nlevels, nkeypoints = (int)keypoints.size();
		bool sortedByLevel = true;

		if( !do_keypoints )
		{
			// if we have pre-computed keypoints, they may use more levels than it is set in parameters
			// !!!TODO!!! implement more correct method, independent from the used keypoint detector.
			// Namely, the detector should provide correct size of each keypoint. Based on the keypoint size
			// and the algorithm used (i.e. BRIEF, running on 31x31 patches) we should compute the approximate
			// scale-factor that we need to apply. Then we should cluster all the computed scale-factors and
			// for each cluster compute the corresponding image.
			//
			// In short, ultimately the descriptor should
			// ignore octave parameter and deal only with the keypoint size.
			nLevels = 0;
			for( i = 0; i < nkeypoints; i++ )
			{
				level = keypoints[i].octave;
				CV_Assert(level >= 0);
				if( i > 0 && level < keypoints[i-1].octave )
					sortedByLevel = false;
				nLevels = std::max(nLevels, level);
			}
			nLevels++;
		}

		std::vector<Rect> layerInfo(nLevels);
		std::vector<int> layerOfs(nLevels);
		std::vector<float> layerScale(nLevels);
		Mat imagePyramid, maskPyramid;
		UMat uimagePyramid, ulayerInfo;

		int level_dy = image.rows + border*2;
		Point level_ofs(0,0);
		Size bufSize((image.cols + border*2 + 15) & -16, 0);

		for( level = 0; level < nLevels; level++ )
		{
			float scale = getScale(level, firstLevel, scaleFactor);
			layerScale[level] = scale;
			Size sz(cvRound(image.cols/scale), cvRound(image.rows/scale));
			Size wholeSize(sz.width + border*2, sz.height + border*2);
			if( level_ofs.x + wholeSize.width > bufSize.width )
			{
				level_ofs = Point(0, level_ofs.y + level_dy);
				level_dy = wholeSize.height;
			}

			Rect linfo(level_ofs.x + border, level_ofs.y + border, sz.width, sz.height);
			layerInfo[level] = linfo;
			layerOfs[level] = linfo.y*bufSize.width + linfo.x;
			level_ofs.x += wholeSize.width;
		}
		bufSize.height = level_ofs.y + level_dy;

		imagePyramid.create(bufSize, CV_8U);
		if( !mask.empty() )
			maskPyramid.create(bufSize, CV_8U);

		Mat prevImg = image, prevMask = mask;

		// Pre-compute the scale pyramids
		for (level = 0; level < nLevels; ++level)
		{
			Rect linfo = layerInfo[level];
			Size sz(linfo.width, linfo.height);
			Size wholeSize(sz.width + border*2, sz.height + border*2);
			Rect wholeLinfo = Rect(linfo.x - border, linfo.y - border, wholeSize.width, wholeSize.height);
			Mat extImg = imagePyramid(wholeLinfo), extMask;
			Mat currImg = extImg(Rect(border, border, sz.width, sz.height)), currMask;

			if( !mask.empty() )
			{
				extMask = maskPyramid(wholeLinfo);
				currMask = extMask(Rect(border, border, sz.width, sz.height));
			}

			// Compute the resized image
			if( level != firstLevel )
			{
				resize(prevImg, currImg, sz, 0, 0, INTER_LINEAR);
				if( !mask.empty() )
				{
					resize(prevMask, currMask, sz, 0, 0, INTER_LINEAR);
					if( level > firstLevel )
						threshold(currMask, currMask, 254, 0, THRESH_TOZERO);
				}

				copyMakeBorder(currImg, extImg, border, border, border, border,
							   BORDER_REFLECT_101+BORDER_ISOLATED);
				if (!mask.empty())
					copyMakeBorder(currMask, extMask, border, border, border, border,
								   BORDER_CONSTANT+BORDER_ISOLATED);
			}
			else
			{
				copyMakeBorder(image, extImg, border, border, border, border,
							   BORDER_REFLECT_101);
				if( !mask.empty() )
					copyMakeBorder(mask, extMask, border, border, border, border,
								   BORDER_CONSTANT+BORDER_ISOLATED);
			}
			prevImg = currImg;
			prevMask = currMask;
		}


		if( do_keypoints )
		{

			// Get keypoints, those will be far enough from the border that no check will be required for the descriptor
			computeKeyPoints(imagePyramid,maskPyramid,
							 layerInfo, ulayerInfo, layerScale, keypoints,vpairlines_all,
							 nfeatures, scaleFactor, edgeThreshold, patchSize, scoreType, fastThreshold);
		}
		else
		{
			KeyPointsFilter::runByImageBorder(keypoints, image.size(), edgeThreshold);

			if( !sortedByLevel )
			{
				std::vector<std::vector<KeyPoint> > allKeypoints(nLevels);
				nkeypoints = (int)keypoints.size();
				for( i = 0; i < nkeypoints; i++ )
				{
					level = keypoints[i].octave;
					CV_Assert(0 <= level);
					allKeypoints[level].push_back(keypoints[i]);
				}
				keypoints.clear();
				for( level = 0; level < nLevels; level++ )
					std::copy(allKeypoints[level].begin(), allKeypoints[level].end(), std::back_inserter(keypoints));
			}
		}

		if( do_descriptors )
		{
			int dsize = descriptorSize();

			nkeypoints = (int)keypoints.size();
			if( nkeypoints == 0 )
			{
				_descriptors.release();
				return;
			}

			_descriptors.create(nkeypoints, dsize, CV_8U);
			std::vector<Point> pattern;

			const int npoints = 512;
			Point patternbuf[npoints];
			const Point* pattern0 = (const Point*)bit_pattern_31_;

			if( patchSize != 31 )
			{
				pattern0 = patternbuf;
				makeRandomPattern(patchSize, patternbuf, npoints);
			}

			CV_Assert( wta_k == 2 || wta_k == 3 || wta_k == 4 );

			if( wta_k == 2 )
				std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));
			else
			{
				int ntuples = descriptorSize()*4;
				initializeOrbPattern(pattern0, pattern, ntuples, wta_k, npoints);
			}

			for( level = 0; level < nLevels; level++ )
			{
				// preprocess the resized image
				Mat workingMat = imagePyramid(layerInfo[level]);

				//boxFilter(working_mat, working_mat, working_mat.depth(), Size(5,5), Point(-1,-1), true, BORDER_REFLECT_101);
				GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);
			}


			{
				Mat descriptors = _descriptors.getMat();
				computeOrbDescriptors(imagePyramid, layerInfo, layerScale,
									  keypoints, descriptors, pattern, dsize, wta_k);
			}
		}
	}


	// String ORB::getDefaultName() const
	// {
	// 	return (Feature2D::getDefaultName() + ".ORB");
	// }

	double ORB_Impl_LJL::calculateAngle(const cv::line_descriptor::KeyLine& line1, const cv::line_descriptor::KeyLine& line2) {
		// 获取 line1 的方向向量
		double dx1 = line1.endPointX - line1.startPointX;
		double dy1 = line1.endPointY - line1.startPointY;

		// 获取 line2 的方向向量
		double dx2 = line2.endPointX - line2.startPointX;
		double dy2 = line2.endPointY - line2.startPointY;

		// 计算两个向量的点积
		double dotProduct = dx1 * dx2 + dy1 * dy2;

		// 计算两个向量的长度
		double length1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
		double length2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

		// 计算夹角的余弦值
		double cosTheta = dotProduct / (length1 * length2);

		// 确保余弦值在 [-1, 1] 之间
		cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

		// 计算夹角（以弧度表示）
		double angle = std::acos(cosTheta);
		angle = angle*(180.0/CV_PI);
		return angle; // 返回夹角（弧度）
	}

	void ORB_Impl_LJL::drawPartiallyConnectedLine(Mat Img, string imgname, vLIL fans)
	{
		int intersectionlines = fans.size();
		cv::Point2f vertices[4];
		for(int i  = 0; i < intersectionlines; i++)
		{
			Point2f point  =  fans[i].Point;
			cv::line_descriptor::KeyLine l1 = fans[i].line1;
			cv::line_descriptor::KeyLine l2 = fans[i].line2;

			line(Img, l1.getStartPoint(), l1.getEndPoint(), cv::Scalar(255, 0, 0), 2);
			// Point2f center1;
			// RotatedRect  rotRect;
			// center1.x = (line1_sp.x + line1_ep.x) / 2;
			// center1.y = (line1_sp.y + line1_ep.y) / 2;
			// float dy = line1_ep.y - line1_sp.y;
			// float dx = line1_ep.x - line1_sp.x;
			// float degAng = fastAtan2(dy, dx);
			// float arcAng  = degAng / 180 * CV_PI;
			// float length = abs(tan(arcAng)) > 1 ? abs(dy) : abs(dx);
			// CvSize tsize;
			// tsize.height = 20 * 2;
			// tsize.width  = length + 2 * 20;
			// rotRect =  RotatedRect(center1, tsize, degAng);
			// rotRect.points(vertices);
			// for (int i = 0; i < 4; i++) {
			// cv::line(Img, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0), 2);
			// }
			// cv::putText(Img, to_string(i), center1, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
			line(Img, l2.getStartPoint(), l2.getEndPoint(), cv::Scalar(0, 255, 0), 2);
			// Point2f center2 = (line2_sp + line2_ep)*0.5f;
			// cv::putText(Img, to_string(i), center2, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
			circle(Img, point, 5, cv::Scalar(0, 0, 255),-1);
			// cv::putText(Img, to_string(i), point, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
		}
		imshow(imgname, Img);
		// waitKey(20);
	}
	void ORB_Impl_LJL::keyLinesToMat(const std::vector<cv::line_descriptor::KeyLine>& keylines, cv::Mat& mLines) {
		// 将矩阵的行数设置为键线的数量，列数设置为4（每条线有两个点，每个点有两个坐标）
		mLines.create(keylines.size(), 4, CV_32F);

		// 遍历键线向量，将每条线段的起点和终点坐标存入矩阵中的每一行
		for (size_t i = 0; i < keylines.size(); ++i) {
			const auto& line = keylines[i];
			// 起点坐标
			mLines.at<float>(i, 0) = line.startPointX;
			mLines.at<float>(i, 1) = line.startPointY;
			// 终点坐标
			mLines.at<float>(i, 2) = line.endPointX;
			mLines.at<float>(i, 3) = line.endPointY;
		}
	}

	ORB_Impl_LJL::vLIL ORB_Impl_LJL::convertFansToKeyLines(const cv::Mat& fans, const std::vector<cv::line_descriptor::KeyLine>& mLines,int level) {
		std::vector<cv::line_descriptor::KeyLine> keyLines;
		vLIL LILs;
		LIL temp;
		for (int i = 0; i < fans.rows; ++i) {
			// Extract information from fans matrix
			float x = fans.at<float>(i, 0);
			float y = fans.at<float>(i, 1);
			temp.Point = Point2f(x,y);
			int index1 = static_cast<int>(fans.at<float>(i, 2));
			int index2 = static_cast<int>(fans.at<float>(i, 3));

			// Get start and end points of the corresponding lines
			cv::line_descriptor::KeyLine line1 = mLines[index1];
			cv::line_descriptor::KeyLine line2 = mLines[index2];
			// line1.octave = level;
			// line2.octave = level;
			temp.idx1 = index1;
			temp.idx2 = index2;
			temp.line1 = line1;
			temp.line2 = line2;
			temp.angle = static_cast<float>(calculateAngle(line1,line2));
			// std::cout<<"angle: "<<temp.angle<<std::endl;
			LILs.push_back(temp);
		}

		return LILs;
	}

	void ORB_Impl_LJL::drawLILMatches(Mat& img1, Mat& img2, vector<strFanMatch>& vstrFanMatch, vector<strPointMatch>& vstrPointMatch, Mat& outImg)
	{
		// 将两张图片水平叠加成一张大图
		int imgHeight = max(img1.rows, img2.rows);
		int imgWidth = img1.cols + img2.cols;
		outImg = Mat::zeros(imgHeight, imgWidth, CV_8UC3);

		// 在输出图像上绘制原始的两张图片
		img1.copyTo(outImg(Rect(0, 0, img1.cols, img1.rows)));
		img2.copyTo(outImg(Rect(img1.cols, 0, img2.cols, img2.rows)));

		for(size_t i = 0; i < vstrPointMatch.size(); i++)
		{
			Point2f pt1 = vstrPointMatch[i].point1;
			Point2f pt2 = vstrPointMatch[i].point2;
			pt2.x += img1.cols;
			// cv::line_descriptor::KeyLine line1 = vstrFanMatch[i].fserial1
			line(outImg, pt1, pt2, Scalar(0,255,0), 1, LINE_AA);
		}
		imshow("匹配", outImg);
		waitKey(20);
	}
	// 计算角平分线的方向角度
	float ORB_Impl_LJL::computeAngle(const KeyLine& kl1, const KeyLine& kl2) {
		Point2f direction1(kl1.endPointX - kl1.startPointX, kl1.endPointY - kl1.startPointY);
		Point2f direction2(kl2.endPointX - kl2.startPointX, kl2.endPointY - kl2.startPointY);

		Point2f bisector((direction1.x + direction2.x) / 2, (direction1.y + direction2.y) / 2);
		return atan2(bisector.y, bisector.x) * 180 / CV_PI;
	}

	void ORB_Impl_LJL::my_getPointsonPolarline(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2, Mat_<double> F, double T, bool *pbIsKept) {
		vector<Point2f> tempSet1,tempSet2;
		tempSet1 = PointSet1;
		tempSet2 = PointSet2;
		PointSet1.clear();
		PointSet2.clear();
		double a,b,c,dist;
		Mat_<double> homoPoint,lineCoeffs;
		Point2f refPoint,testPoint;
		for (unsigned i = 0; i < tempSet1.size(); i ++)
		{
			refPoint  = tempSet1[i];
			testPoint = tempSet2[i];
			homoPoint = (Mat_<double>(3,1) << refPoint.x, refPoint.y, 1);
			lineCoeffs = F * homoPoint;
			a = lineCoeffs(0);
			b = lineCoeffs(1);
			c = lineCoeffs(2);

			dist = fabs(a*testPoint.x + b*testPoint.y + c)/sqrt(a*a + b*b);
			if (dist <= T)
			{
				PointSet1.push_back(refPoint);
				PointSet2.push_back(testPoint);
				pbIsKept[i] = 1;
			}
			else
			{
				pbIsKept[i] = 0;
			}
		}
	}

	void ORB_Impl_LJL::my_findRobustFundamentalMat(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2, Mat &FMat, bool *pbIsKept) {
		unsigned i;
		vector<uchar> status;
		// findFundamentalMat(PointSet1,PointSet2,CV_RANSAC, _fmatThr, 0.99, status);
		findFundamentalMat(PointSet1,PointSet2,CV_FM_RANSAC, 3.0, 0.99, status);
		int time = 0;
		vector<Point2f> goodPoints1,goodPoints2;
		for (i = 0; i < status.size(); i ++)
		{
			if (status[i] == 1)
			{
				goodPoints1.push_back(PointSet1[i]);
				goodPoints2.push_back(PointSet2[i]);
			}
		}
		// Mat F = findFundamentalMat(goodPoints1, goodPoints2,CV_LMEDS, _fmatThr, 0.99, status);
		Mat F = findFundamentalMat(goodPoints1, goodPoints2,CV_FM_LMEDS, 3.0, 0.99, status);
		my_getPointsonPolarline(PointSet1, PointSet2, F, 3.0, pbIsKept);
		// FMat = findFundamentalMat(PointSet1, PointSet2,CV_LMEDS, _fmatThr, 0.99, status);
		FMat = findFundamentalMat(PointSet1, PointSet2,CV_FM_LMEDS, 3.0, 0.99, status);
		FMat.convertTo(FMat, CV_32F);
	}
	void ORB_Impl_LJL::saveKeypoints(const vector<KeyPoint>& keypoints, const string& filename) {
		ofstream file(filename);
		if (!file.is_open()) {
			cerr << "Could not open the file: " << filename << endl;
			return;
		}

		file << "pt.x,pt.y,angle,size,response,class_id,octave" << endl;
		for (const auto& kp : keypoints) {
			file << kp.pt.x << ","
				 << kp.pt.y << ","
				 << kp.angle << ","
				 << kp.size << ","
				 << kp.response << ","
				 << kp.class_id << ","
				 << kp.octave << endl;
		}

		file.close();
		cout << "Keypoints saved to " << filename << endl;
	}
	void ORB_Impl_LJL::loadKeypoints(const string& filename, vector<KeyPoint>& keypoints_3) {
		ifstream file(filename);
		if (!file.is_open()) {
			cerr << "Could not open the file: " << filename << endl;
			return;
		}

		string line;
		getline(file, line); // 跳过表头
		while (getline(file, line)) {
			stringstream ss(line);
			string item;

			vector<string> tokens;
			while (getline(ss, item, ',')) {
				tokens.push_back(item);
			}

			if (tokens.size() >= 3) {
				float x = stof(tokens[0]);
				float y = stof(tokens[1]);
				float angle = stof(tokens[2]);

				KeyPoint orb;
				orb.pt.x = x;
				orb.pt.y = y;
				// orb.angle = angle;
				keypoints_3.push_back(orb);
			}
		}

		file.close();
		cout << "Keypoints loaded from " << filename << endl;
	}
	// 保存描述子到文件
	void ORB_Impl_LJL::saveDescriptors(const Mat& descriptors, const string& filename) {
		FileStorage fs(filename, FileStorage::WRITE);
		if (!fs.isOpened()) {
			cerr << "Could not open the file: " << filename << endl;
			return;
		}
		fs << "descriptors" << descriptors;
		fs.release();
		cout << "Descriptors saved to " << filename << endl;
	}

	// 从文件中加载描述子
	void ORB_Impl_LJL::loadDescriptors(const string& filename, Mat& descriptors) {
		FileStorage fs(filename, FileStorage::READ);
		if (!fs.isOpened()) {
			cerr << "Could not open the file: " << filename << endl;
			return;
		}
		fs["descriptors"] >> descriptors;
		fs.release();
		cout << "Descriptors loaded from " << filename << endl;
	}

	void ORB_Impl_LJL::saveLILGathersAllToFile(const std::vector<vPL>& LIL_gathers_all, const std::string& filename) {
		std::ofstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Unable to open file " << filename << std::endl;
			return;
		}

		for (const vPL& linePairGroup : LIL_gathers_all) {
			for (const pairline& linePair : linePairGroup) {
				file << "KeyLine1: [" << linePair.KeyLine1.startPointX << ", " << linePair.KeyLine1.startPointY << ", "
					 << linePair.KeyLine1.endPointX << ", " << linePair.KeyLine1.endPointY << "]\n";
				file << "KeyLine2: [" << linePair.KeyLine2.startPointX << ", " << linePair.KeyLine2.startPointY << ", "
					 << linePair.KeyLine2.endPointX << ", " << linePair.KeyLine2.endPointY << "]\n";
				file << "Desc: " << linePair.Desc << "\n";
				file << "Angle: " << linePair.angle << "\n";
				file << "Intersection: [" << linePair.intersection.x << ", " << linePair.intersection.y << "]\n";
				file << "Idx1: " << linePair.idx1 << "\n";
				file << "Idx2: " << linePair.idx2 << "\n";
				file << "Idx_point: " << linePair.idx_point << "\n";
				file << "----------------------------------------\n";
			}
			file << "========================================\n";
		}

		file.close();
	}

	void ORB_Impl_LJL::saveVpairlineLevelToFile(const vector<pair<int,pairline>>& vpairline_level, const std::string& filename) {
		std::ofstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Unable to open file " << filename << std::endl;
			return;
		}

		for (const auto& pair : vpairline_level) {
			int level = pair.first;
			const pairline& linePair = pair.second;

			file << "Level: " << level << "\n";
			file << "KeyLine1: [" << linePair.KeyLine1.startPointX << ", " << linePair.KeyLine1.startPointY << ", "
				 << linePair.KeyLine1.endPointX << ", " << linePair.KeyLine1.endPointY << "]\n";
			file << "KeyLine2: [" << linePair.KeyLine2.startPointX << ", " << linePair.KeyLine2.startPointY << ", "
				 << linePair.KeyLine2.endPointX << ", " << linePair.KeyLine2.endPointY << "]\n";
			file << "Desc: " << linePair.Desc << "\n";
			file << "Angle: " << linePair.angle << "\n";
			file << "Intersection: [" << linePair.intersection.x << ", " << linePair.intersection.y << "]\n";
			file << "Idx1: " << linePair.idx1 << "\n";
			file << "Idx2: " << linePair.idx2 << "\n";
			file << "Idx_point: " << linePair.idx_point << "\n";
			file << "----------------------------------------\n";
		}

		file.close();
	}

	void ORB_Impl_LJL::saveVpairlineImagePyramidToFile(const vector<vector<pair<int,pairline>>>& vpairline_image_pyramid, const std::string& filename) {
		std::ofstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Unable to open file " << filename << std::endl;
			return;
		}

		for (size_t i = 0; i < vpairline_image_pyramid.size(); ++i) {
			file << "Pyramid Level: " << i << "\n";
			for (const auto& pair : vpairline_image_pyramid[i]) {
				int level = pair.first;
				const pairline& linePair = pair.second;

				file << "  Level: " << level << "\n";
				file << "  KeyLine1: [" << linePair.KeyLine1.startPointX << ", " << linePair.KeyLine1.startPointY << ", "
					 << linePair.KeyLine1.endPointX << ", " << linePair.KeyLine1.endPointY << "]\n";
				file << "  KeyLine2: [" << linePair.KeyLine2.startPointX << ", " << linePair.KeyLine2.startPointY << ", "
					 << linePair.KeyLine2.endPointX << ", " << linePair.KeyLine2.endPointY << "]\n";
				file << "  Desc: " << linePair.Desc << "\n";
				file << "  Angle: " << linePair.angle << "\n";
				file << "  Intersection: [" << linePair.intersection.x << ", " << linePair.intersection.y << "]\n";
				file << "  Idx1: " << linePair.idx1 << "\n";
				file << "  Idx2: " << linePair.idx2 << "\n";
				file << "  Idx_point: " << linePair.idx_point << "\n";
				file << "  ----------------------------------------\n";
			}
		}

		file.close();
	}

	void ORB_Impl_LJL::saveVpairlineAllToFile(const vector<pair<int,pairline>>& vpairline_all, const std::string& filename) {
		std::ofstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Unable to open file " << filename << std::endl;
			return;
		}

		for (const auto& pair : vpairline_all) {
			int level = pair.first;
			const pairline& linePair = pair.second;

			file << "Level: " << level << "\n";
			file << "  KeyLine1: [" << linePair.KeyLine1.startPointX << ", " << linePair.KeyLine1.startPointY << ", "
				 << linePair.KeyLine1.endPointX << ", " << linePair.KeyLine1.endPointY << "]\n";
			file << "  KeyLine2: [" << linePair.KeyLine2.startPointX << ", " << linePair.KeyLine2.startPointY << ", "
				 << linePair.KeyLine2.endPointX << ", " << linePair.KeyLine2.endPointY << "]\n";
			file << "  Desc: " << linePair.Desc << "\n";
			file << "  Angle: " << linePair.angle << "\n";
			file << "  Intersection: [" << linePair.intersection.x << ", " << linePair.intersection.y << "]\n";
			file << "  Idx1: " << linePair.idx1 << "\n";
			file << "  Idx2: " << linePair.idx2 << "\n";
			file << "  Idx_point: " << linePair.idx_point << "\n";
			file << "----------------------------------------\n";
		}

		file.close();
	}
	ORB_Impl_LJL::vLIL ORB_Impl_LJL::ExactJunctionFromImage(Mat img,int level) {
		// 创建直线检测器(左)
		vLIL temp;
		Mat mLines1;
		cv::Ptr<cv::line_descriptor::LSDDetector> lsd = cv::line_descriptor::LSDDetector::createLSDDetector();
		std::vector<cv::line_descriptor::KeyLine> keylines1;
		keylines1.clear();
		lsd->detect(img, keylines1, 1.2, 1);
		optimizeAndMergeLines_lsd(keylines1, img);
		keyLinesToMat(keylines1, mLines1);
		CPartiallyRecoverConnectivity p1(mLines1,expandWidth_level[level],fans1,img,fanThr);
		temp = convertFansToKeyLines(fans1,keylines1,level);
		// keylines1.clear();
		return temp;
	}

	ORB_Impl_LJL::vPL ORB_Impl_LJL::ExactPairlinesFromJuction(Mat Img,int level) {
		vLIL LIL_g = ExactJunctionFromImage(Img, level);
		pairline temp;
		vPL vpairline;
		// vpairline.clear();
		// vpairline.resize(LIL_g.size());
		for(int i = 0; i < LIL_g.size(); i++) {
			temp.intersection = LIL_g[i].Point;
			temp.idx1 = LIL_g[i].idx1;
			temp.idx2 = LIL_g[i].idx2;
			// temp.idx_point = i;
			temp.KeyLine1 = LIL_g[i].line1;
			temp.KeyLine2 = LIL_g[i].line2;
			temp.angle = LIL_g[i].angle;
			vpairline.push_back(temp);
		}
		return vpairline;
	}

void ORB_Impl_LJL::checkLineExtremes( cv::Vec4f& extremes, cv::Size imageSize )
	{

		if( extremes[0] < 0 )
			extremes[0] = 0;

		if( extremes[0] >= imageSize.width )
			extremes[0] = (float)imageSize.width - 1.0f;

		if( extremes[2] < 0 )
			extremes[2] = 0;

		if( extremes[2] >= imageSize.width )
			extremes[2] = (float)imageSize.width - 1.0f;

		if( extremes[1] < 0 )
			extremes[1] = 0;

		if( extremes[1] >= imageSize.height )
			extremes[1] = (float)imageSize.height - 1.0f;

		if( extremes[3] < 0 )
			extremes[3] = 0;

		if( extremes[3] >= imageSize.height )
			extremes[3] = (float)imageSize.height - 1.0f;
	}

	ORB_Impl_LJL::vPL ORB_Impl_LJL::foroctave(vPL& temp,int level, const vector<float> & layer, int& max_id,const Mat& imagePyramid,const std::vector<Rect>& layerInfo) {
		vPL s;
		int current_id = max_id;
		for(int i = 0; i < temp.size(); i++) {
			temp[i].KeyLine1.octave = level;
			temp[i].KeyLine1.class_id = temp[i].idx1;
			cv::Vec4f extremes1 = Vec4f(temp[i].KeyLine1.startPointX,temp[i].KeyLine1.startPointY,temp[i].KeyLine1.endPointX,temp[i].KeyLine1.endPointY);
			checkLineExtremes(extremes1,imagePyramid(layerInfo[level]).size());
			temp[i].KeyLine1.startPointX = extremes1[0]*layer[level];
			temp[i].KeyLine1.startPointY = extremes1[1]*layer[level];
			temp[i].KeyLine1.endPointX = extremes1[2]*layer[level];
			temp[i].KeyLine1.endPointY = extremes1[3]*layer[level];
			temp[i].KeyLine1.sPointInOctaveX = extremes1[0];
			temp[i].KeyLine1.sPointInOctaveY = extremes1[1];
			temp[i].KeyLine1.ePointInOctaveX = extremes1[2];
			temp[i].KeyLine1.ePointInOctaveY = extremes1[3];
			temp[i].KeyLine1.lineLength = (float) sqrt( pow( extremes1[0] - extremes1[2], 2 ) + pow( extremes1[1] - extremes1[3], 2 ) );
			// temp[i].KeyLine1.startPointX = temp[i].KeyLine1.sPointInOctaveX*layer[level];
			// temp[i].KeyLine1.startPointY = temp[i].KeyLine1.sPointInOctaveY*layer[level];
			// temp[i].KeyLine1.endPointX = temp[i].KeyLine1.ePointInOctaveX*layer[level];
			// temp[i].KeyLine1.endPointY = temp[i].KeyLine1.ePointInOctaveY*layer[level];

			temp[i].KeyLine2.octave = level;
			temp[i].KeyLine2.class_id =  temp[i].idx2;
			cv::Vec4f extremes2 = Vec4f(temp[i].KeyLine2.startPointX,temp[i].KeyLine2.startPointY,temp[i].KeyLine2.endPointX,temp[i].KeyLine2.endPointY);
			temp[i].KeyLine2.startPointX = extremes2[0]*layer[level];
			temp[i].KeyLine2.startPointY = extremes2[1]*layer[level];
			temp[i].KeyLine2.endPointX = extremes2[2]*layer[level];
			temp[i].KeyLine2.endPointY = extremes2[3]*layer[level];
			temp[i].KeyLine2.sPointInOctaveX = extremes2[0];
			temp[i].KeyLine2.sPointInOctaveY = extremes2[1];
			temp[i].KeyLine2.ePointInOctaveX = extremes2[2];
			temp[i].KeyLine2.ePointInOctaveY = extremes2[3];
			temp[i].KeyLine2.lineLength = (float) sqrt( pow( extremes2[0] - extremes2[2], 2 ) + pow( extremes2[1] - extremes2[3], 2 ) );
			// temp[i].KeyLine2.startPointX = temp[i].KeyLine2.sPointInOctaveX*layer[level];
			// temp[i].KeyLine2.startPointY = temp[i].KeyLine2.sPointInOctaveY*layer[level];
			// temp[i].KeyLine2.endPointX = temp[i].KeyLine2.ePointInOctaveX*layer[level];
			// temp[i].KeyLine2.endPointY = temp[i].KeyLine2.ePointInOctaveY*layer[level];
			temp[i].idx_point = i;

			temp[i].idx_point = ++current_id;
			s.push_back(temp[i]);
		}
		max_id = current_id;
		return s;
	}

	vector<KeyPoint> ORB_Impl_LJL::LILtoVectorKeypoint(vLIL a) {
		KeyPoint temp;
		vector<KeyPoint> Key_points;
		for(int i = 0; i < a.size(); i++) {
			temp.pt = a[i].Point;
			temp.class_id = i;
			Key_points.push_back(temp);
		}
		return Key_points;
	}

	vector<KeyPoint> ORB_Impl_LJL::PairlinetoVectorKeypoint(const vPL& a, int& max_class_id) {
		KeyPoint temp;
		vector<KeyPoint> Key_points;
		Key_points.resize(a.size());
		int current_max_class_id = max_class_id;
		for(int i = 0; i < a.size(); i++) {
			temp.pt = a[i].intersection;
			temp.class_id = ++current_max_class_id;
			//计算角度
			// temp.angle = computeAngle(a[i].KeyLine1,a[i].KeyLine2);
			Key_points.push_back(temp);
		}
		max_class_id = current_max_class_id;
		return Key_points;
	}

	vector<pair<int,ORB_Impl_LJL::pairline>> ORB_Impl_LJL::GetPairline(const vPL& a, const vector<KeyPoint>& b) {
		pair<int,pairline> temp;
		vector<pair<int,ORB_Impl_LJL::pairline>> A;
		// A.clear();
		// A.resize(a.size());
		for(int j = 0; j < a.size(); j++) {
			for(int i = 0; i < b.size(); i++) {
				if(a[j].idx_point == b[i].class_id) {
					temp.first = j;
					temp.second = a[j];
					A.push_back(temp);
				}
			}
		}
		return A;
	}

	vector<pair<int,ORB_Impl_LJL::pairline>> ORB_Impl_LJL::PairlineFilter(vector<pair<int,pairline>>& temp,vector<KeyPoint> key_points) {
		vector<pair<int,pairline>> a;
		a.resize(key_points.size());
		for(int i = 0; i < temp.size(); i++) {
			for(int j = 0; j < key_points.size(); j++) {
				if(temp[i].first == key_points[j].class_id)
					a.push_back(temp[i]);
			}
		}
		return a;
	}

	ORB_Impl_LJL::ORB_Impl_LJL(int _nlevels) :
		nfeatures(500), scaleFactor(1.2f), nlevels(8),
		edgeThreshold(31), firstLevel(0), wta_k(2),
		scoreType(ORB::HARRIS_SCORE), patchSize(31), fastThreshold(20) {
		expandWidth_level.resize(_nlevels);
		expandWidth_level[0] = expandWidth;
		for(int i=1; i<nlevels; i++)
		{
			expandWidth_level[i]=expandWidth/getScale(i,firstLevel,scaleFactor);
		}

	}
	int ORB_Impl_LJL::descriptorSize() const
	{
		return ORB::kBytes;
	}

	int ORB_Impl_LJL::descriptorType() const
	{
		return CV_8U;
	}

	int ORB_Impl_LJL::defaultNorm() const
	{
		return NORM_HAMMING;
	}

	void ORB_Impl_LJL::detectfromimages(InputArray image,
							std::vector<KeyPoint>& keypoints,vector<pair<int,pairline>>& vPL_all1,
							OutputArray descriptors) {

		if( image.empty() ) {
			keypoints.clear();
			return;
		}

		detectAndCompute(image, noArray(), keypoints, vPL_all1,descriptors, false);
		// detectAndCompute(image, noArray(), keypoints, vPL_all2, descriptors, true);

	}

}