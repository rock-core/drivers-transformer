#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include "Transformation.hpp"
#include "TransformationElement.hpp"
#include "TransformationStatus.hpp"

#include <Eigen/Geometry>
#include <string>
#include <base/Time.hpp>
#include <aggregator/StreamAligner.hpp>
#include <map>
#include <boost/bind.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace transformer
{
    /**
     * This class represents a static transformation
     * */
    class StaticTransformationElement : public TransformationElement
    {
    public:
        StaticTransformationElement(const std::string &sourceFrame, const std::string &targetFrame, const TransformationType &transform) : TransformationElement(sourceFrame, targetFrame), staticTransform(transform) {};
        virtual ~StaticTransformationElement() {};
        
        virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& tr)
        {
            tr = staticTransform;
                tr.time = atTime;
            return true;
        };
        
        virtual bool isStatic() { return true; }

    private:
        TransformationType staticTransform;
    };

    /**
     * This class represents a dynamic transformation
     * */
    class DynamicTransformationElement : public TransformationElement
    {
    public:
        DynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame, aggregator::StreamAligner& aggregator, int priority = -10);
        virtual ~DynamicTransformationElement();
        
        virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& result);
        virtual bool isStatic() { return false; }
        
        int getStreamIdx() const
        {
            return streamIdx;
        }
    
    private:
        void aggregatorCallback(const base::Time &ts, const TransformationType &value); 

        aggregator::StreamAligner &aggregator;
        base::Time lastTransformTime;
        TransformationType lastTransform;
        bool gotTransform;
        int streamIdx;
    };

    /**
     * This class represents an inverse transformation.
     * */
    class InverseTransformationElement : public TransformationElement
    {
    public:
        InverseTransformationElement(TransformationElement *source): TransformationElement(source->getTargetFrame(), source->getSourceFrame()), nonInverseElement(source) {};
        virtual ~InverseTransformationElement() {};
        virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& tr);
        virtual bool isStatic() { return nonInverseElement->isStatic(); }

        virtual void addTransformationChangedCallback(boost::function<void (const base::Time &ts)> callback) 
        {
            nonInverseElement->addTransformationChangedCallback(callback);
        };
    
        TransformationElement* getElement();
        TransformationElement const* getElement() const;
    private:
        TransformationElement *nonInverseElement;
    };


    class  TransformationNode;

    /**
     * A class that can be used to get a transformation chain from a set of TransformationElements
     * */
    class TransformationTree
    {
        public:
        ///default constructor
        TransformationTree() : maxSeekDepth(20) {};

        /** Returns the number of registered elements in the tree, as a (static
         * elements, dynamic elements) pair
         */
        std::pair<int, int> getElementsCount() const;
        
        /**
         * Adds a TransformationElement to the set of available elements.
         * 
         * Note, internal the TransformationTree will also add an inverse
         * transformation.
         * */
        void addTransformation(TransformationElement *element);
        
        /**
         * This function tries to generate a transformationChain from 'from' to 'to'.
         * 
         * To generate the transformationChain this function will spann a tree of
         * transformations originating from 'from'. The function will then perform a
         * breadth-first search until it either finds a chain, the tree can't be expanded
         * further, or the search depth is deeper than maxSeekDepth.
         * 
         * In case a chain was found the function returns true and the chain is stored in result.
         * */
        bool getTransformationChain(std::string from, std::string to, std::vector<TransformationElement *> &result);
        
        /**
         * Returns a vector of all currently registered transformation elements.
         * */
        std::vector<TransformationElement *> &getAvailableElements()
        {
            return availableElements;
        }
            
        /**
         * Destructor, deletes all TransformationElements saved in availableElements
         * */
        ~TransformationTree();	
        
        /**
         * Deletes all available TransformationElements
         * */
        void clear();

        /** Dumps information about this tree on LOG_DEBUG_S
         */
        void dumpTree() const;
        
    private:
        ///maximum seek depth while trying to find a transformationChain
        const int maxSeekDepth;

        /**
         * This function will expand the given node.
         * 
         * To expand the node, the function will look up all available TransformationElements
         * add new node for those, where the sourceFrame matches from
         * */
        void addMatchingTransforms(std::string from, transformer::TransformationNode* node);
        
        /**
         * Seeks through childs of the node and looks for a node that has frame 'to'
         * 
         * If found the function returns an iterator the the child.
         * If not returns an iterator pointing to node.childs.end()
         * */
        std::vector< TransformationNode* >::const_iterator checkForMatchingChildFrame(const std::string &to, const TransformationNode &node);
        
        /// List of available transformation elements
        std::vector<TransformationElement *> availableElements;
    };

    /**
     * A class that provides transformations to given samples, ordered in time.
     * */
    class Transformer
    {
        protected:
        aggregator::StreamAligner aggregator;
        std::map<std::pair<std::string, std::string>, int> transformToStreamIndex;
        std::vector<Transformation *> transformations;
        TransformationTree transformationTree;
        int priority;
        TransformerStatus transformerStatus;

        void recomputeAvailableTransformations();
        
    public:
        /** 
         * Default constructor for a transformer
         *
         * @param priority - stream priority which is given to dynamic transform streams.
         */
        Transformer( int priority = -10 ) 
            : priority( priority ) {};
        
        /**
         * Deletes all dynamic and static transformations
         * that are known to the transformer.
         * 
         * Also deletes all samples in the data streams
         * 
         * Callbacks and setups for the data streams are NOT deleted by this method.
         * */
        virtual void clear();
        
        /**
         * This function may be used to manually set a transformation chain.
         * 
         * The function seeks through all registered sample streams, and sets the given 
         * chain to those streams, where the source and target frame matches.
         * */
        void addTransformationChain(std::string from, std::string to, const std::vector<TransformationElement *> &chain);
        
        /**
         * This function registers a wanted transformation at the transformation stack.
         * 
         * It returns a reference to an object that represents the wanted transformation.
         * 
         * Note, the time of the transformation advances if one calls step() on the
         * transformer.
         *
         * @param sourceFrame  The source frame of the transformation
         * @param targetFrame  The target frame of the transformation
         * @return  a Transformation; it is valid to take the address of
         *          this object.
         * */
        Transformation &registerTransformation(std::string sourceFrame, std::string targetFrame);

        /**
         * Returns a vector of the registered transformations.
         *
         * @return a list of all transformations
         * */
        const std::vector<Transformation *> &getRegisteredTransformations()
        {
            return transformations;
        }

        /**
         * Unregisteres a transformation from the transfromation stack.
         * 
         * This removes and deletes the given transformation
         *
         * @param transformation  The transformation to be removed. This
         *                        transformation must have been obtained using
         *                        registerTransformation() or
         *                        getRegisteredTransformations(). All
         *                        references to this object become invalid.
         * */
        void unregisterTransformation(Transformation *transformation);
            
        /**
         * Registers a callback that will be called every time a new transformation is available 
         * for the given Transformation handle.
         * 
         * @sa Transformation::registerUpdateCallback
         *
         * @param transform  The transformation to associate the callback with.
         *                   This transformation must have been obtained using
         *                   registerTransformation() or
         *                   getRegisteredTransformations().
         * @param callback   The callback to be called.
         * */
        void registerTransformCallback(Transformation &transform , boost::function<void (const base::Time &ts, const Transformation &t)> callback) 
        {
            transform.registerUpdateCallback(boost::bind( callback, _1, boost::ref(transform) ));
        }

        /**
         * This function registes a new data stream together with an callback. 
         * 
         * The callback will be called every time a new data sample is available.
         *
         * @sa aggregator::StreamAligner::registerStream
         *
         * @param dataPeriod  time between sensor readings. This will be used
         *                    to estimate when the next reading should arrive,
         *                    so out of order arrivals are possible. Set to 0
         *                    if not a periodic stream. When set to a negative
         *                    value, the calculation of the buffer is performed
         *                    for that period, however no lookahead is set.
         * @param callback    will be called for data gone through the
         *                    synchronization process
         * @param priority    if streams have data with equal timestamps, the
         *                    one with the lower priority value will be pushed
         *                    first.
         * @param name        name of the stream. This is only for debug
         *                    purposes
         * @result            stream index, which is used to identify the
         *                    stream (e.g. for push).
         * */
        template <class T> int registerDataStream(base::Time dataPeriod, boost::function<void (const base::Time &ts, const T &value)> callback, int priority = -1, const std::string &name = std::string())
        {
            return aggregator.registerStream<T>(boost::bind( callback, _1, _2), 0, dataPeriod, priority, name);
        };

        /**
         * This function registes a new data stream together with an callback. 
         * 
         * The callback will be called every time a new data sample is available.
         *
         * @sa aggregator::StreamAligner::registerStream
         *
         * @param dataPeriod      time between sensor readings. This will be
         *                        used to estimate when the next reading should
         *                        arrive, so out of order arrivals are
         *                        possible. Set to 0 if not a periodic stream.
         *                        When set to a negative value, the calculation
         *                        of the buffer is performed for that period,
         *                        however no lookahead is set.
         * @param transformation  The transformation associated with the
         *                        callback.
         * @param callback        will be called for data gone through the
         *                        synchronization process
         * @param priority        if streams have data with equal timestamps,
         *                        the one with the lower priority value will be
         *                        pushed first.
         * @param name            name of the stream. This is only for debug
         *                        purposes
         * @result                stream index, which is used to identify the
         *                        stream (e.g. for push).
         * */
        template <class T> int registerDataStreamWithTransform(base::Time dataPeriod, Transformation &transformation, boost::function<void (const base::Time &ts, const T &value, const Transformation &t)> callback, int priority = - 1, const std::string &name = std::string())
        {
            return aggregator.registerStream<T>(boost::bind( callback, _1, _2, boost::ref(transformation) ), 0, dataPeriod, priority, name);
        };

        /**
         * This function will remove the stream with the given index from the
         * stream aligner.
         *
         * @param idx index of the stream that should be unregistered
         * */
        void unregisterDataStream(int idx)
        {
            return aggregator.unregisterStream(idx);
        };

        /** 
         * Will disable the stream with the given index.
         *
         * All data left in the stream will still be played out, however the
         * stream will be ignored for lookahead and timeout calculation. A
         * stream, which is disabled can be enabled through the enableStream()
         * call, or if new data in this stream arrives.
         *
         * The functionality is needed for cases where streams might be
         * optional, so that the other streams won't be delayed up to the
         * maximum time out value.
         *
         * @param idx index of the stream that should be disabled
         */
        void disableStream( int idx )
        {
            return aggregator.disableStream( idx );
        }

        /** 
         * Enables a stream which has been disabled previously.
         *
         * All streams are enabled by default. Does not have any effect on
         * streams which are already enabled.
         *
         * @param idx index of the stream that should be enabled
         */
        void enableStream( int idx )
        {
            return aggregator.enableStream( idx );
        }

        /** 
         * See if a stream is enabled or disabled.
         *
         * @param idx index of the stream to look at
         * @return true if the stream is enabled (active)
         */
        bool isStreamActive( int idx ) const 
        {
            return aggregator.isStreamActive( idx );
        }
        
        /**
         * @deprecated
         *
         * This was required to use a previous version of
         * registerTransformCallback(registerTransfromCallback)
         * that created a bool typed stream.
         *
         * The current implementation of registerTransformCallback
         * no longer requires a call to requestTransformationAtTime.
         *
         * This only works with indexes of bool typed streams.
         *
         * See commit 299857530298fd43912d341cdd273dc20c83fd9a,
         * a35ede8dd0f84dfefb36627033c1df5c72b56ff1
         */
#if defined(__cplusplus) && (__cplusplus >= 201402L)
        [[deprecated("No longer needed with registerTransformCallback")]]
#elif defined(__GNUC__) || defined(__clang__)
        __attribute__((deprecated("No longer needed with registerTransformCallback")))
#endif
        void requestTransformationAtTime(int idx, base::Time ts)
        {
            aggregator.push(idx, ts, false);
        };
        
        /** Push new data into the stream
         *
         * @param idx   The stream index, as obtained from
         *              registerDataStream or registerDataStreamWithTransform
         * @param ts    the timestamp of the data item
         * @param data  the data added to the stream
         */
        template <class T> void pushData( int idx,const base::Time &ts, const T& data )
        {
            aggregator.push(idx, ts, data);
        };
        
        /** This will go through the available streams and look for the
         * oldest available data. The data can be either existing are predicted
         * through the period.
         *
         * There are three different cases that can happen:
         *  - The data is already available. In this case that data is forwarded
         *    to the callback.
         *  - The data is not yet available, and the time difference between oldest
         *    data and newest data is below the timeout threshold. In this case
         *    no data is called.
         *  - The data is not yet available, and the timeout is reached. In this
         *    case, the oldest data (which is obviously non-available) is ignored,
         *    and only newer data is considered.
         *
         *  @result - true if a callback was called and more data might be available
         * */
        int step()
        {
            return aggregator.step();
        }
        
        /** @return the current status of the StreamAligner
         * this is mainly used for debug purposes
         */
        const aggregator::StreamAlignerStatus &getStreamAlignerStatus()
        {
            return aggregator.getStatus();
        }
        
        /** Set the time the Estimator will wait for an expected reading on any of the streams.
         * This number effectively puts an upper limit to the lag that can be created due to
         * delay or missing values on the channels.
         */
        void setTimeout(const base::Time &t )
        {
            aggregator.setTimeout(t);
        }
        
        /**
         * Function for adding new Transformation samples.
         *
         * This function will interally keep track of available 
         * transformations and register streams for 'new'
         * transformations.  
         *
         * @param tr  The dynamic transformation to integrate
         * */
        virtual void pushDynamicTransformation(const TransformationType &tr);
        
        /**
         * Function for adding static Transformations.
         *
         * @param tr  The state transformation to integrate
         * */
        void pushStaticTransformation(const TransformationType &tr);

        
        /**
         * Renames a frame
         *
         * @param frameName  The old name of the frame
         * @param newName    The new name of the frame
         * */
        void setFrameMapping(const std::string &frameName, const std::string &newName);

        /** @return the current status of the StreamAligner
         * this is mainly used for debug purposes
         */
        const aggregator::StreamAlignerStatus& getStatus()
        {
            return aggregator.getStatus();
        }
        
        /** 
         * @return the status of the transformer
         */
        const TransformerStatus& getTransformerStatus();
    
        /**
         * Destructor, deletes the TransformationMakers
         * */
        virtual ~Transformer();
    };
}
#endif // TRANSFORMER_H
