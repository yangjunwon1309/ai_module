#!/bin/bash
VS_PKG_PATH=$(rospack find vehicle_simulator)
exec "$VS_PKG_PATH/mesh/unity/environment/Model.x86_64" -force-glcore
