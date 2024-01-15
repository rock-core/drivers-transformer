#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <string>
#include <vector>

#include "TransformationElement.hpp"
#include "TransformationStatus.hpp"

namespace transformer
{
    /** The transformation between two frames
     *
     * Objects of this type can be used to get the current relative
     * transformation between the involved frames.
     *
     * These are constructed by Transformer::registerTransformation
     */
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
        *
        * @param chain  The transformation chain
        * */
        void setTransformationChain(const std::vector<TransformationElement *> &chain);

        Transformation(const Transformation &other){}

    public:
        /** Returns a status structure with data derived from the
         * transformation's internal information
        * @returns The updated status structure
        */
        TransformationStatus getStatus() const;

        /** Updates the data contained in the provided status structure with the
        * transformation's internal information
        *
        * @param status  The status structure to be updated.
        */
        void updateStatus(TransformationStatus& status) const;

        /**
        * returns the source frame
        * @return The source frame
        * */
        const std::string &getSourceFrame() const
        {
            if(sourceFrameMapped.empty())
                return sourceFrame;

            return sourceFrameMapped;
        }

        /**
        * returns the target frame
        * @return The target frame
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
        * Registers a callback that is called whenever this transformation changes.
        *
        * There can only be one callback; the last setting is overwritten
        * by this new callback
        *
        * @param callback   The callback to be called. The single parameter
        *                   is the timestamp of the change.
        *                   To unregister the last callback, the parameter
        *                   can be set to
        *                   boost::function<void (const base::Time &ts)>()
        * */
        void registerUpdateCallback(boost::function<void (const base::Time &ts)> callback);

        /**
        * This function tries to return the transformation from sourceFrame to targetFrame at the given time.
        * 
        * If no chain, or no transformation samples are available the function will return false
        * Else it will return true and store the requested transformation in result
        *
        * @sa get(const base::Time& atTime, T& result, bool interpolate) const
        *
        * @param atTime      This is the time that is used to interpolate the
        *                    transformation. It must be equal or newer than the
        *                    latest timestamp reported via the callback, it
        *                    must be older than the next sample from any of the
        *                    involved dynamic transformations.
        *                    atTime is only used to populate result.time
        *                    when interpolate is false.
        * @param result      The calculated transformation, from a freshly
        *                    initialized TransformationType, with sourceFrame
        *                    and targetFrame set to the configured values and
        *                    time set to atTime.
        * @param interpolate If set to true, the final transformation will be
        *                    calculated from interpolated versions of all of
        *                    the involved dynamic transformations.
        *                    Else, the the latest samples that are older or
        *                    same age as the time reported through the callback
        *                    will be used.
        * @return            True if a transformation was produced.
        * */
        bool get(const base::Time& atTime, transformer::TransformationType& result, bool interpolate = false) const;

        /**
        * This function tries to return the transformation from sourceFrame to targetFrame at the given time.
        *
        * If no chain, or no transformation samples are available the function will return false
        * Else it will return true and store the requested transformation in result
        *
        * @param atTime      This is the time that is used to interpolate the
        *                    transformation. It must be equal or newer than the
        *                    latest timestamp reported via the callback, it
        *                    must be older than the next sample from any of the
        *                    involved dynamic transformations.
        *                    atTime is only used to fill the time member
        *                    variables when interpolate is false.
        * @param result      The calculated transformation chain. The time
        *                    members will be set to atTime
        * @param interpolate If set to true, the final transformation will be
        *                    calculated from interpolated versions of all of
        *                    the involved dynamic transformations.
        *                    Else, the the latest samples that are older or
        *                    same age as the time reported through the callback
        *                    will be used.
        * @return            True if a transformation was produced.
        * */
        bool getChain(const base::Time& atTime, std::vector<TransformationType>& result, bool interpolate = false) const;

        /**
        * This function tries to return the transformation from sourceFrame to targetFrame at the given time.
        *
        * If no chain, or no transformation samples are available the function will return false
        * Else it will return true and store the requested transformation in result
        *
        * @sa get(const base::Time& atTime, transformer::TransformationType& result, bool interpolate) const
        *
        * @param atTime      This is the time that is used to interpolate the
        *                    transformation. It must be equal or newer than the
        *                    latest timestamp reported via the callback, it
        *                    must be older than the next sample from any of the
        *                    involved dynamic transformations.
        *                    atTime is ignored when interpolate is false.
        * @param result      The calculated transformation, from a freshly
        *                    initialized T.
        * @param interpolate If set to true, the final transformation will be
        *                    calculated from interpolated versions of all of
        *                    the involved dynamic transformations.
        *                    Else, the the latest samples that are older or
        *                    same age as the time reported through the callback
        *                    will be used.
        * @return            True if a transformation was produced.
        * */
        template <class T>
        bool get(const base::Time& atTime, T& result, bool interpolate = false) const;

        /**
        *
        * \copydoc getChain(const base::Time&,std::vector<TransformationType>&,bool) const
        * */
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