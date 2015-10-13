##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=VOCollisionAvoidance
ConfigurationName      :=Debug
WorkspacePath          := "C:\codelitewp\cppwp"
ProjectPath            := "C:\codelitewp\cppwp\VOCollisionAvoidance"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=valentin
Date                   :=10/14/15
CodeLitePath           :="C:\Program Files\CodeLite"
LinkerName             :=g++
SharedObjectLinkerName :=g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.o.i
DebugSwitch            :=-gstab
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E 
ObjectsFileList        :="VOCollisionAvoidance.txt"
PCHCompileFlags        :=
MakeDirCommand         :=makedir
RcCmpOptions           := 
RcCompilerName         :=windres
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := ar rcus
CXX      := g++
CC       := gcc
CXXFLAGS :=  -g -O0 -Wall -std=c++11 $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall $(Preprocessors)
ASFLAGS  := 
AS       := as


##
## User defined environment variables
##
CodeLiteDir:=C:\Program Files\CodeLite
Objects0=$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/Tester.cpp$(ObjectSuffix) $(IntermediateDirectory)/ORCASolver.cpp$(ObjectSuffix) $(IntermediateDirectory)/CPLPSolver.cpp$(ObjectSuffix) $(IntermediateDirectory)/MathUtils.cpp$(ObjectSuffix) $(IntermediateDirectory)/SVGExporter.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

$(IntermediateDirectory)/.d:
	@$(MakeDirCommand) "./Debug"

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/main.cpp$(ObjectSuffix): main.cpp $(IntermediateDirectory)/main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/codelitewp/cppwp/VOCollisionAvoidance/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(DependSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/main.cpp$(DependSuffix) -MM "main.cpp"

$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix) "main.cpp"

$(IntermediateDirectory)/Tester.cpp$(ObjectSuffix): Tester.cpp $(IntermediateDirectory)/Tester.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/codelitewp/cppwp/VOCollisionAvoidance/Tester.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/Tester.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Tester.cpp$(DependSuffix): Tester.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Tester.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/Tester.cpp$(DependSuffix) -MM "Tester.cpp"

$(IntermediateDirectory)/Tester.cpp$(PreprocessSuffix): Tester.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Tester.cpp$(PreprocessSuffix) "Tester.cpp"

$(IntermediateDirectory)/ORCASolver.cpp$(ObjectSuffix): ORCASolver.cpp $(IntermediateDirectory)/ORCASolver.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/codelitewp/cppwp/VOCollisionAvoidance/ORCASolver.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/ORCASolver.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/ORCASolver.cpp$(DependSuffix): ORCASolver.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/ORCASolver.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/ORCASolver.cpp$(DependSuffix) -MM "ORCASolver.cpp"

$(IntermediateDirectory)/ORCASolver.cpp$(PreprocessSuffix): ORCASolver.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/ORCASolver.cpp$(PreprocessSuffix) "ORCASolver.cpp"

$(IntermediateDirectory)/CPLPSolver.cpp$(ObjectSuffix): CPLPSolver.cpp $(IntermediateDirectory)/CPLPSolver.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/codelitewp/cppwp/VOCollisionAvoidance/CPLPSolver.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/CPLPSolver.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/CPLPSolver.cpp$(DependSuffix): CPLPSolver.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/CPLPSolver.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/CPLPSolver.cpp$(DependSuffix) -MM "CPLPSolver.cpp"

$(IntermediateDirectory)/CPLPSolver.cpp$(PreprocessSuffix): CPLPSolver.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/CPLPSolver.cpp$(PreprocessSuffix) "CPLPSolver.cpp"

$(IntermediateDirectory)/MathUtils.cpp$(ObjectSuffix): MathUtils.cpp $(IntermediateDirectory)/MathUtils.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/codelitewp/cppwp/VOCollisionAvoidance/MathUtils.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MathUtils.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MathUtils.cpp$(DependSuffix): MathUtils.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MathUtils.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/MathUtils.cpp$(DependSuffix) -MM "MathUtils.cpp"

$(IntermediateDirectory)/MathUtils.cpp$(PreprocessSuffix): MathUtils.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MathUtils.cpp$(PreprocessSuffix) "MathUtils.cpp"

$(IntermediateDirectory)/SVGExporter.cpp$(ObjectSuffix): SVGExporter.cpp $(IntermediateDirectory)/SVGExporter.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/codelitewp/cppwp/VOCollisionAvoidance/SVGExporter.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/SVGExporter.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/SVGExporter.cpp$(DependSuffix): SVGExporter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/SVGExporter.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/SVGExporter.cpp$(DependSuffix) -MM "SVGExporter.cpp"

$(IntermediateDirectory)/SVGExporter.cpp$(PreprocessSuffix): SVGExporter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/SVGExporter.cpp$(PreprocessSuffix) "SVGExporter.cpp"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


