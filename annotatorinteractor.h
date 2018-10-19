#ifndef ANNOTATORINTERACTOR_H
#define ANNOTATORINTERACTOR_H

#include <pcl/visualization/interactor_style.h>

class AnnotatorInteractor: public pcl::visualization::PCLVisualizerInteractorStyle
{
public:

    static AnnotatorInteractor *New ();


    /** \brief Empty constructor. */
    AnnotatorInteractor () : paint_signal(), pcl::visualization::PCLVisualizerInteractorStyle()
    {}

    /** \brief Empty destructor */
    virtual ~AnnotatorInteractor () {}

    // this macro defines Superclass, the isA functionality and the safe downcast method
    vtkTypeMacro (AnnotatorInteractor, pcl::visualization::PCLVisualizerInteractorStyle)

    boost::signals2::connection
    registerPaintingCallback (boost::function<void (const double, const double, const double, const long, const bool)> cb);

    boost::signals2::signal<void (const double, const double, const double, const long, const bool)> paint_signal;

    void Initialize();


    void OnKeyDown();
    void OnKeyUp();
    void OnMouseMove();

    bool isPainting = false;
    bool isErasing = false;

    double getPainterTolerance() const;
    void setPainterTolerance(double value);

    double getPointPickerTolerance() const;
    void setPointPickerTolerance(double value);

protected:
    double pointPickerTolerance = 0.2;
    double painterTolerance = 0.2;
    vtkSmartPointer<vtkPointPicker> pp;

};

#endif // ANNOTATORINTERACTOR_H
