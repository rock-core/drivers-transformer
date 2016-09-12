#ifndef TRANSFORMATION_ELEMENT_HPP
#define TRANSFORMATION_ELEMENT_HPP

#include <base/samples/RigidBodyState.hpp>

#include <string>
#include <vector>
#include <boost/function.hpp>

namespace transformer
{
    typedef base::samples::RigidBodyState TransformationType;
    
    /**
     * This is a base class, that represens an abstract transformation from sourceFrame to targetFrame. 
     * */
    class TransformationElement
    {
    public:
        TransformationElement(const std::string &sourceFrame, const std::string &targetFrame): sourceFrame(sourceFrame), targetFrame(targetFrame) {};
        virtual ~TransformationElement() {};
        
        /**
         * This method my be asked for a concrete transformation at a time X.
         * On request this should should interpolate the transformation to
         * match the given time better than the available data. 
         * 
         * Note if no transformation is available getTransformation will return false
         * */
        virtual bool getTransformation(const base::Time &atTime, bool doInterpolation, TransformationType &tr) = 0;

        /**
         * This function registers a callback, that should be called every
         * time the TransformationElement changes its value. 
         * */
        virtual void addTransformationChangedCallback(boost::function<void (const base::Time &ts)> callback)
        {
            elementChangedCallbacks.push_back(callback);
        };
        
        /**
         * Whether this transformation is dynamic or static.
         */
        virtual bool isStatic() = 0;
        
        /**
         * Removes all registered callbacks
         * */
        virtual void clearTransformationChangedCallbacks()
        {
            elementChangedCallbacks.clear();
        }
        
        /**
         * returns the name of the source frame
         * */
        const std::string &getSourceFrame() const
        {
            return sourceFrame;
        }
        
        /**
         * returns the name of the target frame
         * */
        const std::string &getTargetFrame() const
        {
            return targetFrame;
        }

    protected:
        std::vector<boost::function<void (const base::Time &ts)> > elementChangedCallbacks;

    private:
        std::string sourceFrame;
        std::string targetFrame;
    };
}

#endif
