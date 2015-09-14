/**
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "g12Buffer.h"
#include "integralBuffer.h"
#include "bufferFactory.h"
#include "cortageListMatcher.h"
#include "cortageStripeMatcher.h"
#include "cortageListSorter.h"
#include "abstractSignatureGenerator.h"
#include "modelSignatureGenerator.h"
#include "signatureBuilders/differencesignaturebuilder.h"
#include "signatureBuilders/differentepsilonsignaturebuilder.h"
#include "signatureBuilders/elementdifferentepssignaturebuilder.h"
#include "signatureBuilders/pattern4signaturesbuilder.h"
#include "signatureBuilders/pattern3signaturebuilder.h"
#include "signatureBuilders/elementsignaturebuilder.h"
#include "adaptiveFeatureGenerator.h"
#include "derivativeBuffer.h"
#include "flowFiltering.h"

using namespace std;

vector<ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> *> getModelFeaturingGenerators()
{
    vector<ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> *> generators;

    LinearFeaturingParameters sParam;
    EpsilonTable3Linear epsilonCalculator(sParam);

    FeaturingBuilder * builder6 = new ElementDifferentEpsFeaturingBuilder();
    ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> * gen6 =
            new ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer>(&epsilonCalculator, builder6);
    generators.push_back(gen6);

    FeaturingBuilder * builder1 = new Pattern4FeatureVectorsBuilder();
    ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> * gen1 =
            new ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer>(&epsilonCalculator, builder1);
    generators.push_back(gen1);

    FeaturingBuilder * builder2 = new Pattern3FeaturingBuilder();
    ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> * gen2 =
            new ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer>(&epsilonCalculator, builder2);
    generators.push_back(gen2);

    FeaturingBuilder * builder3 = new ElementFeaturingBuilder();
    ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> * gen3 =
            new ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer>(&epsilonCalculator, builder3);
    generators.push_back(gen3);

    FeaturingBuilder * builder4 = new DifferenceFeaturingBuilder();
    ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> * gen4 =
            new ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer>(&epsilonCalculator, builder4);
    generators.push_back(gen4);

    FeaturingBuilder * builder5 = new DifferentEpsilonFeaturingBuilder();
    ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> * gen5 =
            new ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer>(&epsilonCalculator, builder5);
    generators.push_back(gen5);


    return generators;
}

vector <AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *>
        getAdaptiveFeaturingGenerators()
{
    LinearFeaturingParameters params(1,10,1,150);
    EpsilonTable3Linear epsilon(params);
    vector <AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *> generators;

    AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *gen1 =
        new AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList>(
        &epsilon, new ElementDifferentEpsFeaturingBuilder());
    generators.push_back(gen1);

    AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *gen2 =
        new AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList>(
        &epsilon, new Pattern4FeatureVectorsBuilder());
    generators.push_back(gen2);

    AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *gen3 =
        new AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList>(
        &epsilon, new Pattern3FeaturingBuilder());
    generators.push_back(gen3);

    AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *gen4 =
        new AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList>(
        &epsilon, new ElementFeaturingBuilder());
    generators.push_back(gen4);

    AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *gen5 =
        new AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList>(
        &epsilon, new DifferenceFeaturingBuilder());
    generators.push_back(gen5);

    AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *gen6 =
        new AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList>(
        &epsilon, new DifferentEpsilonFeaturingBuilder());
    generators.push_back(gen6);

    return generators;
}

void reproduce(G12Buffer *input, CortegeList *cortageList,
               ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> *generator)
{
    FeaturingBuffer *featureVectors = new FeaturingBuffer(input->h, input->w);
    G12IntegralBuffer *integralInput = new G12IntegralBuffer(input);
    generator->generateFeatureVectors(featureVectors,  integralInput);
    featureVectors->reproduce<CortegeList>(cortageList);
    CortegeListSorter::sort(cortageList);
}

void initializeFeatureVector(RectCortegeList *featureVectors, G12Buffer *input,
        AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *generator)
{
    DerivativeBuffer *derivative = new DerivativeBuffer(input);
    G12Buffer *gradMagnitude = derivative->gradientMagnitudeBuffer();
    G12IntegralBuffer *inputInt = new G12IntegralBuffer(input);
    G12IntegralBuffer * gradInt = new G12IntegralBuffer(gradMagnitude);
    G12Buffer *mask;
    featureVectors->fillWith(Cortege(0,Vector2d16(0,0)));
    generator->generateFeatureVectors(featureVectors, inputInt, gradInt, &mask);
    CortegeListSorter::sort(featureVectors);
}

TEST(SignatureCompare, compareFeaturingGenerators)
{
    G12Buffer *input0 = BufferFactory::getInstance()->loadG12Bitmap("../data/pair/image0001_c0.pgm");
    G12Buffer *input1 = BufferFactory::getInstance()->loadG12Bitmap("../data/pair/image0002_c0.pgm");

    CortegeListMatcher matcher(7);
    vector<ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> *> generators =
            getModelFeaturingGenerators();
    for (unsigned i = 0; i < generators.size(); i ++) {
        ModelFeaturingGenerator<EpsilonTable3Linear, FeaturingBuffer> * generator = generators.at(i);

        CortegeList *sig0 = new CortegeList(input0->h, input0->w);
        reproduce(input0, sig0, generator);
        CortegeList *sig1 = new CortegeList(input1->h, input1->w);
        reproduce(input1, sig1, generator);

        FlowBuffer *flowUnfiltered = matcher.doMatch(sig0,sig1);
        FlowBuffer *flow = FlowFiltering::filterFlowSimple(flowUnfiltered, 7, 9, 11);
        int density = flow->density();
        printf("%s: tr = %6d %f \n", generator->toString(), density,
               (double) density / (input0->h * input0->w));
        delete flow;
        delete flowUnfiltered;
        delete sig0;
        delete sig1;
    }

    vector <AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *>
            adaptiveGenerators  = getAdaptiveFeaturingGenerators();
    for (unsigned i = 0; i < adaptiveGenerators.size(); i ++) {
        AdaptiveFeaturingGenerator<EpsilonTable3Linear, RectCortegeList> *generator =
                adaptiveGenerators.at(i);
        generator->addLevel(ActionLevel(2,300,2,2));
        generator->addLevel(ActionLevel(2,200,4,4));
        generator->addLevel(ActionLevel(2,100,6,6));

        RectCortegeList *featureVectors0 = new RectCortegeList(input0->h, input0->w, 5, 5);
        RectCortegeList *featureVectors1 = new RectCortegeList(input1->h, input1->w, 5, 5);
        initializeFeatureVector(featureVectors0, input0, generator);
        initializeFeatureVector(featureVectors1, input1, generator);

        FlowBuffer *flowUnfiltered = matcher.doMatch(featureVectors0, featureVectors1);
        FlowBuffer *flow = FlowFiltering::filterFlowSimple(flowUnfiltered, 7, 9, 11);
        int density = flow->density();
        printf("%s: tr = %6d %f \n", generator->toString(), density,
               (double) density / (input0->h * input0->w));
        delete flow;
        delete flowUnfiltered;
    }
    delete input0;
    delete input1;

}


int main (int /*argC*/, char **/*argV*/)
{
    cout << "PASSED" << endl;
    compareFeaturingGenerators();
    return 0;
}
