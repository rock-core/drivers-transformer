#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <string>
#include <vector>

#include "TransformationElement.hpp"
#include "TransformationStatus.hpp"

namespace transformer
{
    class Transformation
    {
    friend class Transformer;

    private:
        Transformation(const std::string &sourceFrame, const std::string &targetFrame)
        : valid(false)
        , sourceFrame(sourceFrame)
        , targetFrame(targetFrame)
        , generatedTransformations(0)
        , failedNoChain(0)
        , failedNoSample(0)
        , failedInterpolationImpossible(0) {};
        bool valid;

        std::string sourceFrame;
        std::string targetFrame;
        std::string sourceFrameMapped;
        std::string targetFrameMapped;
        std::vector<TransformationElement *> transformationChain;

        mutable base::Time lastGeneratedValue;
        mutable uint64_t generatedTransformations;
        mutable uint64_t failedNoChain;
        mutable uint64_t failedNoSample;
        mutable uint64_t failedInterpolationImpossible;
        boost::function<void (const base::Time &ts)> transformationChangedCallback;

        void setFrameMapping(const std::string &frameName, const std::string &newName);

        /**
        * Sets the transformation chain for this transformation
        *
        * The transformation chain is a list of links (represented by
        * TransformationElement objects) that should be composed to compute the
        * required transformation
        *
        * Calling this method sets the transformation as valid.
        * */
        void setTransformationChain(const std::vector<TransformationElement *> &chain);

        Transformation(const Transformation &other){}

    public:
        /** Updates the data contained in the provided status structure with the
        * transformation's internal information
        */
        TransformationStatus getStatus() const;

        /** Updates the data contained in the provided status structure with the
        * transformation's internal information
        */
        void updateStatus(TransformationStatus& status) const;

        /**
        * returns the souce frame
        * */
        const std::string &getSourceFrame() const
        {
            if(sourceFrameMapped.empty())
                return sourceFrame;

            return sourceFrameMapped;
        }

        /**
        * returns the target frame
        * */
        const std::string &getTargetFrame() const
        {
            if(targetFrameMapped.empty())
                return targetFrame;

            return targetFrameMapped;
        }	

        /** Clears all stored information and marks the transformation as
        * invalid
        */
        void reset()
        {
            valid = false;
            transformationChain.clear();
            lastGeneratedValue = base::Time();
            generatedTransformations = 0;
            failedNoChain = 0;
            failedNoSample = 0;
            failedInterpolationImpossible = 0;
        }

        /**
        * Registeres a callback (only one) callback, that is called,
        * whenever this transformation changes.
        * */
        void registerUpdateCallback(boost::function<void (const base::Time &ts)> callback);

        /**
        * This functions tries to return the transformation from sourceFrame to targetFrame at the given time.
        * 
        * If no chain, or no transformation samples are available the function will return false
        * Else it will return true and store the requested transformation in @param result
        * */
        bool get(const base::Time& atTime, transformer::TransformationType& result, bool interpolate = false) const;
        bool getChain(const base::Time& atTime, std::vector<TransformationType>& result, bool interpolate = false) const;

        template <class T>
        bool get(const base::Time& atTime, T& result, bool interpolate = false) const;
        bool getChain(const base::Time& atTime, std::vector<Eigen::Affine3d>& result, bool interpolate = false) const;
    };


    template<class T>
    bool Transformation::get(const base::Time& atTime, T& result, bool interpolate) const
    {
        result = T::Identity();
        if (!valid)
        {
            failedNoChain++;
            return false;
        }

        for(std::vector<TransformationElement *>::const_iterator it = transformationChain.begin(); it != transformationChain.end(); it++)
        {
        TransformationType tr;
        if(!(*it)->getTransformation(atTime, interpolate, tr))
        {
                if (interpolate)
                    failedInterpolationImpossible++;
                else
                    failedNoSample++;

            //no sample available, return
            return false;
        }
        
        //TODO, this might be a costly operation
        T trans( tr );
        
        //apply transformation
        result = result * trans;
        }
        lastGeneratedValue = atTime;
        generatedTransformations++;
        return true;
    }
}

#endif