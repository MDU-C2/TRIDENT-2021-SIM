
list (APPEND BULLET_LIBRARIES LinearMath)
#list (APPEND BULLET_LIBRARIES Bullet3Common)
#list (APPEND BULLET_LIBRARIES BulletInverseDynamics)
list (APPEND BULLET_LIBRARIES BulletCollision)
list (APPEND BULLET_LIBRARIES BulletDynamics)
#list (APPEND BULLET_LIBRARIES BulletSoftBody)

set(BULLET_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/external/stonefish/3rdparty)
