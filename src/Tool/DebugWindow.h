
#pragma once

/// VTK
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkCommand.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkCubeSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkGlyph3D.h>

/// USER
#include "DataType.h"
#include "SharedStorage.h"

namespace SCV
{

class DebugWindow
{

public:
    DebugWindow() { ren = vtkRenderer::New(); renWindow = vtkRenderWindow::New(); renWindowInteractor = vtkRenderWindowInteractor::New(); }
    ~DebugWindow() {}

protected:
    vtkRenderer *ren;
    vtkRenderWindow *renWindow;
    vtkRenderWindowInteractor *renWindowInteractor;

};

class MapViewer : public DebugWindow
{
    class MapViewerTimerCallback : public vtkCommand
    {

    public:
        static MapViewerTimerCallback *New(MapViewer *newViewer)
        {
            MapViewerTimerCallback *newTimerCallback = new MapViewerTimerCallback;

            newTimerCallback->viewer = newViewer;

            return newTimerCallback;
        }

        virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData))
        {
            if (vtkCommand::TimerEvent == eventId)
            {
                viewer->update();
            }
        }

    private:
        MapViewer *viewer;

    };


public:
    MapViewer(SharedStorage *newStorage)
    {
        map = 0;
        storage = newStorage;
        imgArr = vtkUnsignedCharArray::New();
        imgData = vtkImageData::New();
        imgActor = vtkImageActor::New();
        interactorStyle = vtkInteractorStyleImage::New();
        tmrCallback = MapViewerTimerCallback::New(this);

        cubeSrc = vtkCubeSource::New();
        mapper = vtkPolyDataMapper::New();
        cubeActor = vtkActor::New();
        cubePolyData = vtkPolyData::New();
        cubeGlyph3D = vtkGlyph3D::New();
        prevAngleZ = 0;
    }
    ~MapViewer()
    {
        imgArr->Delete();
        imgData->Delete();
        imgActor->Delete();
        interactorStyle->Delete();
        tmrCallback->Delete();

        cubeSrc->Delete();
        mapper->Delete();
        cubeActor->Delete();
        cubePolyData->Delete();
        cubeGlyph3D->Delete();
    }

private:
    SharedStorage *storage;
    Map *map;
    vtkUnsignedCharArray *imgArr;
    vtkImageData *imgData;
    vtkImageActor *imgActor;
    vtkInteractorStyleImage *interactorStyle;
    MapViewerTimerCallback *tmrCallback;

    vtkCubeSource *cubeSrc;
    vtkPolyDataMapper *mapper;
    vtkActor *cubeActor;
    vtkPolyData *cubePolyData;
    vtkGlyph3D *cubeGlyph3D;

    double prevAngleZ;

private:
    void mapConnection(Parameter &param)
    {
        Map* newMap = storage->getMapPtr();

        if (newMap == 0) { std::cout << "mapConnection : no mapPtr\n\n"; return; }

        if (map != 0) { std::cout << "mapConnection : already connected\n\n"; return; }

        map = newMap;
        int maxX = map->getMaxX();
        int maxY = map->getMaxY();
        imgArr->SetArray(map->getPixels(), 3 * maxX * maxY, 1);

        imgData->GetPointData()->SetScalars(imgArr);
        imgData->SetScalarType(VTK_UNSIGNED_CHAR);
        imgData->SetNumberOfScalarComponents(3);
        imgData->SetExtent(0, maxX - 1, 0, maxY - 1, 0, 1);
        imgData->SetSpacing(1, 1, 1);
        imgData->SetOrigin(0, 0, 0);

        cubeSrc->SetCenter(param.map_size_x / 2, param.map_size_y / 2, 0);
        cubeSrc->SetBounds((-param.vehicle_length / 2) / param.map_resolution, (param.vehicle_length / 2) / param.map_resolution, (-param.vehicle_width / 2) / param.map_resolution, (param.vehicle_width / 2) / param.map_resolution, -1, 1);

        vtkPoints *cubePoint = vtkPoints::New();
        cubePoint->InsertNextPoint(0,0,0);
        vtkUnsignedCharArray *cubeColor = vtkUnsignedCharArray::New();
        cubeColor->SetName("colors");
        cubeColor->SetNumberOfComponents(3);
        unsigned char r = 255;
        unsigned char g = 0;
        unsigned char b = 0;
        cubeColor->InsertNextTupleValue(&r);
        cubeColor->InsertNextTupleValue(&g);
        cubeColor->InsertNextTupleValue(&b);

        cubePolyData->SetPoints(cubePoint);
        cubePolyData->GetPointData()->SetScalars(cubeColor);

        cubeGlyph3D->SetColorModeToColorByScalar();
        cubeGlyph3D->SetSourceConnection(cubeSrc->GetOutputPort());
        cubeGlyph3D->SetInput(cubePolyData);
        cubeGlyph3D->ScalingOff();
        cubeGlyph3D->Update();

        mapper->SetInputConnection(cubeGlyph3D->GetOutputPort());
        cubeActor->SetMapper(mapper);

        imgActor->SetInput(imgData);
        imgActor->InterpolateOff();

        renWindow->AddRenderer(ren);
        renWindow->SetSize(maxX, maxY);

        ren->AddActor(imgActor);
        ren->AddActor(cubeActor);
        renWindowInteractor->SetInteractorStyle(interactorStyle);
        renWindowInteractor->SetRenderWindow(renWindow);
        renWindowInteractor->Initialize();


        /// timer callback
        renWindowInteractor->AddObserver(vtkCommand::TimerEvent, tmrCallback);
        int timerID = renWindowInteractor->CreateRepeatingTimer(30);
    }

public:
    void start(Parameter &param)
    {
        mapConnection(param);
        renWindowInteractor->Start();
    }

    void update()
    {
        if (storage->exit())
        {
            renWindowInteractor->GetRenderWindow()->Finalize();
            renWindowInteractor->TerminateApp();
            return;
        }

        if (storage->updateMap())
        {
            imgData->Modified();
            imgData->Update();
            imgActor->Modified();
        }
        Pose pose3d;
        storage->getVehiclePose(pose3d);
        cubeActor->SetPosition(pose3d.x, pose3d.y, 0);
        cubeActor->RotateZ((pose3d.az - prevAngleZ) * 180 / M_PI);
        prevAngleZ = pose3d.az;
        cubeActor->Modified();

        renWindow->Render();
    }

};





}
