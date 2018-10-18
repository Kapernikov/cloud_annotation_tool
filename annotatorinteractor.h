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
    registerPaintingCallback (boost::function<void (const double, const double, const double, const long)> cb);

    boost::signals2::signal<void (const double, const double, const double, const long)> paint_signal;

    void Initialize();


    void OnKeyDown();
    void OnKeyUp();
    void OnMouseMove();

    bool isPainting = false;
};

#endif // ANNOTATORINTERACTOR_H
