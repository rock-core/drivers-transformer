#include "Transformation.hpp"

using namespace transformer;

bool Transformation::get(const base::Time &time, TransformationType& tr, bool doInterpolation) const
{
    tr.initSane();
    tr.sourceFrame = sourceFrame;
    tr.targetFrame = targetFrame;
    tr.time = time;

    Eigen::Affine3d fullTransformation;
    bool ret = get(time, fullTransformation, doInterpolation);
    if(!ret)
        return false;

    tr.setTransform(fullTransformation);
    return true;
}

bool Transformation::getChain(const base::Time& time, std::vector< TransformationType >& tr, bool doInterpolation) const
{
    if(transformationChain.empty()) 
        return false;

    tr.resize(transformationChain.size());
    TransformationType transform;
    int i = 0;
    for(std::vector<TransformationElement *>::const_iterator it = transformationChain.begin(); it != transformationChain.end(); it++)
    {
        tr[i].sourceFrame = (*it)->getSourceFrame();
        tr[i].targetFrame = (*it)->getTargetFrame();
        tr[i].time = time;

        //no sample available, return
        if(!(*it)->getTransformation(time, doInterpolation, tr[i]))
            return false;

        i++;
    }
    return true;
}

bool Transformation::getChain(const base::Time& atTime, std::vector< Eigen::Affine3d >& result, bool interpolate) const
{
    std::vector< TransformationType > intern;
    bool ret = getChain(atTime, intern, interpolate);
    if(!ret)
        return false;

    result.resize(intern.size());
    std::vector<Eigen::Affine3d >::iterator it_out = result.begin();
    for(std::vector<TransformationType >::const_iterator it = intern.begin(); it != intern.end(); it++)
    {
        *it_out = *it;
        it_out++;
    }

    return true;
}

void Transformation::setFrameMapping(const std::string& frameName, const std::string& newName)
{
    if(sourceFrame == frameName)
        sourceFrameMapped = newName;

    if(targetFrame == frameName)
        targetFrameMapped = newName;
}

TransformationStatus Transformation::getStatus() const
{
    TransformationStatus status;
    updateStatus(status);
    return status;
}

void Transformation::updateStatus(TransformationStatus& status) const
{
    if (status.source_local != sourceFrame)
        status.source_local = sourceFrame;
    if (status.target_local != targetFrame)
        status.target_local = targetFrame;
    if (status.source_global != getSourceFrame())
        status.source_global = getSourceFrame();
    if (status.target_global != getTargetFrame())
        status.target_global = getTargetFrame();
    status.last_generated_value = lastGeneratedValue;
    status.chain_length = transformationChain.size();
    status.generated_transformations = generatedTransformations;
    status.failed_no_sample = failedNoSample;
    status.failed_no_chain = failedNoChain;
    status.failed_interpolation_impossible = failedInterpolationImpossible;
}

void Transformation::setTransformationChain(const std::vector< TransformationElement* >& chain)
{
    transformationChain = chain;
    valid = true;

    if(!transformationChangedCallback.empty())
    {
        for(std::vector< TransformationElement* >::iterator it = transformationChain.begin();
        it != transformationChain.end(); it++)
        {
            if((*it)->isStatic())
            {
                //call the callback, as the transformation will never 'change' again 
                transformationChangedCallback(base::Time());
            }else
            {
                // add the callback for the dynamic transformation
                (*it)->addTransformationChangedCallback(transformationChangedCallback);
            }
        }
    }
}

void Transformation::registerUpdateCallback(boost::function<void (const base::Time &ts)> callback)
{
    transformationChangedCallback = callback;
    if(valid)
    {
        for(std::vector< TransformationElement* >::iterator it = transformationChain.begin();
        it != transformationChain.end(); it++)
        {
            (*it)->addTransformationChangedCallback(transformationChangedCallback);
        }
    }
}