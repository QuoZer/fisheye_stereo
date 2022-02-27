#include "FisheyeDewarper.hpp"
#include "Stereopair.hpp"

/*
 - User adds cameras with their relative positions
 - User picks camera pairs defining  a stereopair
	* User sets 3D optical axis ?
	* 3D optical axis defined by positions?
 -   



*/


class SurroundSystem {
	//*   Model constants   *//
	const int SCARAMUZZA = -1;
	const int PINHOLE = 00;
	const int SCARAMUZZA = 10;
	const int ATAN = 20;
	const int MEI = 30;

private: //* Containers *//
	std::vector<FisheyeDewarper*> dewarpers; 
	std::vector<CameraModel*> cameras;		// replace with a dictionary?
	std::vector<Stereopair*> stereopairs;

public:
	int addNewCam(CameraModel* readyModel);
	
	int createStereopair(int lCamIndex, int rCamIndex, cv::Size reconstructedRes, cv::Vec3d direction);

	void prepareLUTs();


};