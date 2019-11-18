#pragma once

// base code includes
#include                  <Triangulation.h>
#include                  <Wrapper.h>

using namespace std;

namespace ttk{

  class LevelSets : public Debug{

    public:

        LevelSets();

        ~LevelSets();


        template <class dataType>
        int execute(dataType level, vector<vector<float> >& minima, vector<vector<float> >& maxima, vector<vector<float> >&lines);

        template <class dataType>
        bool findPoint(float f, int v1, int v2, dataType f1, dataType f2, vector<float>& point);

        inline int setInputDataPointer(void *data){
            inputData_ = data;
            return 0;
        }


        inline int setupTriangulation(Triangulation *triangulation){
            triangulation_ = triangulation;

            if(triangulation_){
                triangulation_->preprocessVertexNeighbors();
            }

            return 0;
        }

    protected:

      void                  *inputData_;
      Triangulation         *triangulation_;
  };
}


// template functions
template <class dataType> int ttk::LevelSets::execute(dataType level, vector<vector<float> >& minima, vector<vector<float> >& maxima, vector<vector<float> >& lines){
    //the following line is how we read the input scalar field,
    //we recall that the scalar field here is defined on the vertices of our triangulation
    //so inputData[i] will correspond to the scalar value associated to the vertex of index i
    dataType *inputData = (dataType *) inputData_;

    //STEP 1 - Extract the minima and maxima from your dataset
    //TODO: you need to cycle on the vertices of your triangulation

    //TIP: you can retrieve the total number of vertices with the function
    //triangulation_->getNumberOfVertices();
    bool maxima_flag=true,minima_flag=true;
    int neighbors;
    int min;
    int max;

        min=inputData[0];
        max=inputData[0];
    float x_coOrdinate,y_coOrdinate,z_coOrdinate;
    for(int current_vertex_id=0;current_vertex_id<triangulation_->getNumberOfVertices();current_vertex_id++){
        float parent_val=inputData[current_vertex_id];
        if( inputData[current_vertex_id]<min){
                min=inputData[current_vertex_id];
            }
        else if(inputData[current_vertex_id]> max){
                max=inputData[current_vertex_id];
            }
        for(int i=0;i<triangulation_->getVertexNeighborNumber(current_vertex_id);i++){
            maxima_flag=true;
            minima_flag=true;
            triangulation_->getVertexNeighbor(current_vertex_id,i,neighbors);
            float child_val=inputData[neighbors];
            if(child_val>parent_val){
                maxima_flag=false;                
            }
            if(child_val<parent_val){
           
                minima_flag=false;
            }
            if(!minima_flag && !maxima_flag){
                break;
            }
        }
        if(maxima_flag){
            vector<float> maxima_vector;
            triangulation_->getVertexPoint(current_vertex_id,x_coOrdinate,y_coOrdinate,z_coOrdinate);
            maxima_vector.push_back(x_coOrdinate);
            maxima_vector.push_back(y_coOrdinate);
            maxima_vector.push_back(z_coOrdinate);
            maxima.push_back(maxima_vector);
        }
        if(minima_flag){
            vector<float> minima_vector;
            triangulation_->getVertexPoint(current_vertex_id,x_coOrdinate,y_coOrdinate,z_coOrdinate);
            minima_vector.push_back(x_coOrdinate);
            minima_vector.push_back(y_coOrdinate);
            minima_vector.push_back(z_coOrdinate);        
            minima.push_back(minima_vector);
        }  
    }

    float normalized_value=((level)*(max-min))+min;
    int num_of_triangles=triangulation_->getNumberOfTriangles();

        for(int i=0;i<num_of_triangles;i++){

                int vertex1,vertex2,vertex3;

                triangulation_->getTriangleVertex(i,0,vertex1);
                triangulation_->getTriangleVertex(i,1,vertex2);
                triangulation_->getTriangleVertex(i,2,vertex3);

                vector<float> point1,point2,point3;
                bool v1_v2=findPoint(normalized_value,vertex1,vertex2,inputData[vertex1],inputData[vertex2],point1);
                bool v2_v3=findPoint(normalized_value,vertex2,vertex3,inputData[vertex2],inputData[vertex3],point2);
                bool v3_v1= findPoint(normalized_value,vertex3,vertex1,inputData[vertex3],inputData[vertex1],point3);

                if(v1_v2 && v2_v3){
                    lines.push_back(point1);
                    lines.push_back(point2);

                }
                else if(v2_v3 && v3_v1){
                    lines.push_back(point2);
                    lines.push_back(point3);

                }

                else if(v3_v1 && v1_v2){
                    lines.push_back(point3);
                    lines.push_back(point1);

                }

        }

  return 0;
}

template<class dataType>
bool ttk::LevelSets::findPoint(float lvlSet, int v1, int v2, dataType f1, dataType f2, vector<float> &point) {

    point = vector<float>();
    float x1,y1,z1,x2,y2,z2,xt,yt,zt;

    if((lvlSet > f1 && lvlSet < f2) || (lvlSet > f2 && lvlSet < f1)){
    triangulation_->getVertexPoint(v1,x1,y1,z1);
    triangulation_->getVertexPoint(v2,x2,y2,z2);
    
    float t = (f1-lvlSet)/(f1-f2);
    xt=((1-t)*x1) +(t*x2);
    yt=((1-t)*y1) +(t*y2);
    zt=((1-t)*z1) +(t*z2);

     point.push_back(xt);
     point.push_back(yt);
     point.push_back(zt);

return true;
}
    return false;
}

