ttk_add_paraview_plugin(ttkLevelSets
	SOURCES ${VTKWRAPPER_DIR}/ttkLevelSets/ttkLevelSets.cpp
	PLUGIN_VERSION 1.0
	PLUGIN_XML LevelSets.xml
	LINK levelSets)

