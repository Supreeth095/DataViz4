#include                  <ttkLevelSets.h>

using namespace std;
using namespace ttk;

vtkStandardNewMacro(ttkLevelSets)

int ttkLevelSets::doIt(vector<vtkDataSet *> &inputs, vector<vtkDataSet *> &outputs){

    //this is how we read the input from port 0
    vtkDataSet *input = inputs[0];

    //ttk will creat a triangulation for us, pretty convenient
    Triangulation *triangulation = ttkTriangulation::getTriangulation(input);

    //if something went wrong in initializing the triangulation, stop here
    if(!triangulation)
        return -1;

    //here we are passing the triangulation to the core of our module
    triangulation->setWrapper(this);
    levelSets_.setupTriangulation(triangulation);
    levelSets_.setWrapper(this);

    //read the scalar field from the input dataset
    vtkDataArray *inputScalarField = NULL;

    if(ScalarField.length()){
        inputScalarField = input->GetPointData()->GetArray(ScalarField.data());
    }
    else{
        inputScalarField = input->GetPointData()->GetArray(0);
    }

    //if something went wrong while reading the scalar field, stop here
    if(!inputScalarField)
        return -2;

    //preparing the structures where to store minima, maxima and contour lines
    vector<vector<float> > minima, maxima;
    vector<vector<float> > lines;

    // calling the execute function in the module
    // the switch has the purpouse of adapting the function type to the input type of our scalar field.
    // this way, it does not matter if the input scalar field is defined by integers, doubles or floats,
    // the function will adapt to it automatically.
    levelSets_.setInputDataPointer(inputScalarField->GetVoidPointer(0));
    switch(inputScalarField->GetDataType()){
        ttkTemplateMacro(levelSets_.execute<VTK_TT>(Level, minima, maxima, lines));
    }

    //here we start preparing our output for Paraview
    //we take input port 0 where we will plot maxima and minima
    vtkUnstructuredGrid* outputCP=vtkUnstructuredGrid::SafeDownCast(outputs[0]);

    //we preare a list of points in a format readable for VTK
    vtkSmartPointer<vtkPoints> pairOrigins=vtkSmartPointer<vtkPoints>::New();
    //we prepare a list of values to be associated with each point (a type that will say if the point is a max or a min)
    vtkSmartPointer<vtkCharArray> type=vtkSmartPointer<vtkCharArray>::New();
    //we say that each component of this array will have 1 single value (like a scalar field)
    type->SetNumberOfComponents(1);
    //we set the name of this new field
    type->SetName("Type");

    //for each minimum we prepare point by inserting its three coordinnates
    //then we save its type
    for(int i=0; i<minima.size(); i++){
        pairOrigins->InsertNextPoint(minima[i][0],minima[i][1],minima[i][2]);
        type->InsertNextTuple1(0);
    }

    //for each maximum we prepare a point by inserting its three coordinnates
    //then we save its type
    for(int i=0; i<maxima.size(); i++){
        pairOrigins->InsertNextPoint(maxima[i][0],maxima[i][1],maxima[i][2]);
        type->InsertNextTuple1(1);
    }

    //add the points to the output
    outputCP->SetPoints(pairOrigins);
    //add the field to the output
    vtkPointData* pointData = outputCP->GetPointData();
    pointData->AddArray(type);


    //we take input port 1 where we will plot the contour lines
    vtkUnstructuredGrid* outputLines=vtkUnstructuredGrid::SafeDownCast(outputs[1]);

    //we preare a list of points in a format readable for VTK
    vtkSmartPointer<vtkPoints> linePoints=vtkSmartPointer<vtkPoints>::New();

    //we fill the list of points
    for(int i=0; i<lines.size(); i+=2){
        linePoints->InsertNextPoint(lines[i][0],lines[i][1],lines[i][2]);
        linePoints->InsertNextPoint(lines[i+1][0],lines[i+1][1],lines[i+1][2]);
    }
    //add the points to the output
    outputLines->SetPoints(linePoints);

    //allocate space for the lines
    outputLines->Allocate(lines.size()/2);

    //here we are creating the edges forming the contour lines. For each pair of points computed
    //by our function we create a VTK_LINE that will connect vertices of index i and i+1
    for(int i=0; i<lines.size(); i+=2){
        vtkIdType line[2];
        line[0]=i;
        line[1]=i+1;
        outputLines->InsertNextCell(VTK_LINE, 2, line);
    }

  return 0;
}
