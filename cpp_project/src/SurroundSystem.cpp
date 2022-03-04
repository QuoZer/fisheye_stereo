#include "SurroundSystem.hpp"

// takes cameramodel object with all the parameters already set. Return new camera index
int SurroundSystem::addNewCam(CameraModel* readyModel)
{
	cameras.push_back(readyModel);	
	dewarpers.push_back(&FisheyeDewarper(readyModel));

	return cameras.size()-1;  // new camera index
}

CameraModel SurroundSystem::getCameraModel(CameraModels cm)
{
	// FIXME: antipattern
	switch (cm)
	{
	case PINHOLE:
		return PinholeModel();
		break;
	case ATAN:
		return AtanModel();
		break;
	case SCARAMUZZA:
		return ScaramuzzaModel();
		break;
	case MEI:
		return MeiModel();
		break;
	case KB:
		break;
	default:
		break;
	}
}

int SurroundSystem::createStereopair(int lCamIndex, int rCamIndex, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod sm)
{
	CameraModel *left = cameras[lCamIndex];
	CameraModel *right = cameras[rCamIndex];
	FisheyeDewarper *leftDewarper = dewarpers[lCamIndex];
	FisheyeDewarper *rightDewarper = dewarpers[rCamIndex];
	// TODO: interpolation setter ?? 
	leftDewarper->setSize(left->oldSize, reconstructedRes, 90);  // HACK: 90deg is an assumption
	leftDewarper->setRpy(0, 0, 0);									// TODO: calc angles from position
	rightDewarper->setSize(right->oldSize, reconstructedRes, 90);
	rightDewarper->setRpy(0, 0, 0);

	Stereopair* SP = new Stereopair(left, leftDewarper, right, rightDewarper);
	SP->setOptimalDirecton();
	SP->setStereoMethod(sm);
	stereopairs.push_back(SP);



	return stereopairs.size() - 1;
}


void SurroundSystem::prepareLUTs()
{
	for each (Stereopair* SP in stereopairs)
	{
		SP->fillMaps();
	}
}
