#include "includes.h"
#include "defines.h"
#include "typedefs.h"

void filter(const CloudPtr& , CloudPtr& , float);
void scaleDown(const CloudPtr&, CloudPtr&, float);
Eigen::Matrix4f ICPAlign(const CloudPtr& , const CloudPtr&, Eigen::Matrix4f&, int);