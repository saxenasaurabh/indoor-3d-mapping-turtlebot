#define HSIZE 256 // RGB histogram size
#define VFH_SIZE 180 // Only first 180 bytes of VFH are being considered as we dont want viewpoint dependence
#define RGB_NORMALIZE_SCALE 8
#define VFH_NORMALIZE_SCALE 2
#define DESCRIPTOR_HSIZE (3*(HSIZE/RGB_NORMALIZE_SCALE)+VFH_SIZE/VFH_NORMALIZE_SCALE)
#define planeText "Cloud-%d Plane-%d"
#define clusterText "Cloud-%d Cluster-%d"