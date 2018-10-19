#include "annotatorinteractor.h"
#include <vtkPointPicker.h>
#include <vtkObjectFactory.h>
#include <vtkAssemblyPath.h>
#include <vtkAbstractPicker.h>
#include <vtkCamera.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>


vtkStandardNewMacro(AnnotatorInteractor);

boost::signals2::connection AnnotatorInteractor::registerPaintingCallback(boost::function<void (const double, const double, const double, const long, const bool)> cb)
{
    return (paint_signal.connect (cb));
}

void AnnotatorInteractor::Initialize()
{
    pcl::visualization::PCLVisualizerInteractorStyle::Initialize();
    pp = vtkSmartPointer<vtkPointPicker>::New ();
    pp->SetTolerance (pp->GetTolerance () * 6);
    if (Interactor) {
        Interactor->SetPicker (pp);
    }
}



void AnnotatorInteractor::OnKeyDown()
{
    // Get the status of special keys (Cltr+Alt+Shift)
    bool shift = Interactor->GetShiftKey   ();
    bool ctrl  = Interactor->GetControlKey ();
    bool alt   = Interactor->GetAltKey ();
    if ((Interactor->GetKeySym ()[0] == 'p' || Interactor->GetKeySym ()[0] == 'P') && !ctrl && !alt && !shift) {
        isPainting = true;
    } else if ((Interactor->GetKeySym ()[0] == 'e' || Interactor->GetKeySym ()[0] == 'E') && !ctrl && !alt && !shift) {
        isErasing = true;
    } else {
        pcl::visualization::PCLVisualizerInteractorStyle::OnKeyDown();
    }
}

void AnnotatorInteractor::OnMouseMove()
{
    if (isPainting || isErasing) {
        double cameraPos[4], cameraFP[4];
        vtkCamera *camera;
        camera = this->CurrentRenderer->GetActiveCamera();
        camera->GetPosition(cameraPos);
        cameraPos[3] = 1.0;
        camera->GetFocalPoint(cameraFP);
        cameraFP[3] = 1.0;



        vtkAssemblyPath *path = NULL;
        int x = this->Interactor->GetEventPosition()[0];
        int y = this->Interactor->GetEventPosition()[1];
        vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
        pp->SetTolerance (getPainterTolerance());
        //Interactor->SetPicker (pp);

        pp->Pick (x, y, 0.0, this->CurrentRenderer);
        vtkAbstractPropPicker *picker;
        if ((picker = vtkAbstractPropPicker::SafeDownCast (pp)))
          path = picker->GetPath ();
        if (path != NULL) {
            auto d1 = picker->GetPickPosition()[0];
            auto d2 = picker->GetPickPosition()[1];
            auto d3 = picker->GetPickPosition()[2];
            paint_signal(d1,d2,d3, pp->GetPointId(), isPainting);
        }
    } else {
        PCLVisualizerInteractorStyle::OnMouseMove();
    }
}

double AnnotatorInteractor::getPainterTolerance() const
{
    return painterTolerance;
}

void AnnotatorInteractor::setPainterTolerance(double value)
{
    painterTolerance = value;
}

double AnnotatorInteractor::getPointPickerTolerance() const
{
    return pointPickerTolerance;
}

void AnnotatorInteractor::setPointPickerTolerance(double value)
{
    pointPickerTolerance = value;
    pp->SetTolerance(value);
}

void AnnotatorInteractor::OnKeyUp()
{
    if ((Interactor->GetKeySym ()[0] == 'p' || Interactor->GetKeySym ()[0] == 'P') ) {
        isPainting = false;
    } else if  ((Interactor->GetKeySym ()[0] == 'e' || Interactor->GetKeySym ()[0] == 'E') ) {
        isErasing = false;
    } else {
        pcl::visualization::PCLVisualizerInteractorStyle::OnKeyUp();
    }
}
