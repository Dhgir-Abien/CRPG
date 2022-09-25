//
//  cart.hpp
//  crv_01
//
//  Created by Es1chUb on 2022/8/2.
//  Copyright © 2022 dhgir.abien. All rights reserved.
//

//Purpose: Implementation of Classification and Regression Trees
//From: https://github.com/vidalt/Decision-Trees

#ifndef cart_h
#define cart_h

#include <string>
#include <vector>
#include <set>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <time.h>
#include <math.h>

#define MY_EPSILON 0.00001
enum AttributeType { TYPE_NUMERICAL, TYPE_CATEGORICAL };

class Params
{
public:

    /* GENERAL PARAMETERS */
    std::string pathToInstance ;    // Path to the instance
    std::string pathToSolution ;    // Path to the solution
    
    /* PARAMETERS OF THE ALGORITHM */
    int seed;                        // Random seed (for randomized algorithms)
    clock_t maxTime;                // CPU time limit, if you want to use such a limit in your metaheuristic
    int maxDepth ;                    // Depth limit for the decision tree

    /* DATASET INFORMATION */
    std::string datasetName;                                // Name of the dataset
    int nbSamples;                                            // Number of samples
    int nbAttributes;                                        // Number of attributes
    int nbClasses;                                            // Number of classes
    std::vector <AttributeType> attributeTypes;                // Type of the attributes (TYPE_NUMERICAL or TYPE_CATEGORICAL)
    std::vector < std::vector < double > > dataAttributes;  // Dataset: attributes of each sample
    std::vector < int > dataClasses;                        // Dataset: class of each sample
    std::vector < int > nbLevels;                            // Dataset: number of possible levels/categories (only for categorical attributes)

    /* TO MEASURE CPU TIME */
    clock_t startTime;                // Time when the algorithm started
    clock_t endTime;                // Time when the algorithm ended

    /* CONSTRUCTOR */
    Params(std::string pathToInstance, std::string pathToSolution, int seedRNG, int maxDepth, clock_t maxTime);
    Params(std::vector<std::vector<double> > &dataset, std::vector<std::string> &attributes, std::vector<int> &datasetClasses,
           int seedRNG, int maxDepth, clock_t maxTime, std::string datasetName, int nbClasses);
};

Params::Params(std::string pathToInstance, std::string pathToSolution, int seedRNG, int maxDepth, clock_t maxTime) : seed(seedRNG), pathToInstance(pathToInstance), pathToSolution(pathToSolution), maxDepth(maxDepth), maxTime(maxTime)
{
    // Initializing random number generator here (if you have nondeterministic components)
    //std::srand(seedRNG);
    //std::cout << "----- INITIALIZING RNG WITH SEED: " << seedRNG << std::endl;

    std::ifstream inputFile(pathToInstance.c_str());
    if (inputFile.is_open())
    {
        // Reading the dataset
        std::string useless, attType;
        inputFile >> useless >> datasetName ;
        inputFile >> useless >> nbSamples;
        inputFile >> useless >> nbAttributes;
        inputFile >> useless;
        for (unsigned int i = 0; i < nbAttributes; i++)
        {
            inputFile >> attType;
            if (attType == "C") attributeTypes.push_back(TYPE_CATEGORICAL);
            else if (attType == "N") attributeTypes.push_back(TYPE_NUMERICAL);
            else throw std::string("ERROR: non recognized attribute type");
        }
        inputFile >> useless >> nbClasses;
        dataAttributes = std::vector<std::vector<double> >(nbSamples, std::vector<double>(nbAttributes));
        dataClasses    = std::vector<int>(nbSamples);
        nbLevels = std::vector<int>(nbAttributes,0);
        for (unsigned int s = 0; s < nbSamples; s++)
        {
            for (unsigned int i = 0; i < nbAttributes; i++)
            {
                inputFile >> dataAttributes[s][i];
                if (attributeTypes[i] == TYPE_CATEGORICAL && dataAttributes[s][i]+1 > nbLevels[i])
                    nbLevels[i] = dataAttributes[s][i]+1;
            }
            inputFile >> dataClasses[s];
            if (dataClasses[s] >= nbClasses)
                throw std::string("ERROR: class indices should be in 0...nbClasses-1");
        }
        inputFile >> useless;
        if (!(useless == "EOF"))
            throw std::string("ERROR when reading instance, EOF has not been found where expected");
        std::cout << "----- DATASET [" << datasetName << "] LOADED IN " << clock()/ (double)CLOCKS_PER_SEC << "(s)" << std::endl;
        std::cout << "----- NUMBER OF SAMPLES: " << nbSamples << std::endl;
        std::cout << "----- NUMBER OF ATTRIBUTES: " << nbAttributes << std::endl;
        std::cout << "----- NUMBER OF CLASSES: " << nbClasses << std::endl;
    }
    else
        std::cout << "----- IMPOSSIBLE TO OPEN DATASET: " << pathToInstance << std::endl;
}

Params::Params(std::vector<std::vector<double> > &dataset, std::vector<std::string> &attributes, std::vector<int> &datasetClasses,
               int seedRNG, int maxDepth, clock_t maxTime, std::string datasetName, int nbClasses)
    : seed(seedRNG), maxDepth(maxDepth), maxTime(maxTime), datasetName(datasetName), nbClasses(nbClasses)
{
    nbSamples = dataset.size();
    nbAttributes = attributes.size();
    for(auto &attType : attributes){
        if (attType == "C") attributeTypes.push_back(TYPE_CATEGORICAL);
        else if (attType == "N") attributeTypes.push_back(TYPE_NUMERICAL);
        else throw std::string("ERROR: non recognized attribute type");
    }
    dataAttributes = dataset;
    dataClasses    = datasetClasses;
    nbLevels = std::vector<int>(nbAttributes,0);
    for (unsigned int s = 0; s < nbSamples; s++){
        for (unsigned int i = 0; i < nbAttributes; i++){
            if (attributeTypes[i] == TYPE_CATEGORICAL && dataAttributes[s][i]+1 > nbLevels[i])
                nbLevels[i] = dataAttributes[s][i]+1;
        }
        if (dataClasses[s] >= nbClasses)
            throw std::string("ERROR: class indices should be in 0...nbClasses-1");
    }
    std::cout << "----- DATASET [" << datasetName << "] LOADED" << std::endl;
    std::cout << "----- NUMBER OF SAMPLES: " << nbSamples << std::endl;
    std::cout << "----- NUMBER OF ATTRIBUTES: " << nbAttributes << std::endl;
    std::cout << "----- NUMBER OF CLASSES: " << nbClasses << std::endl;
}

// Structure representing one node of the (orthogonal) decision tree or a leaf
class DTNode
{

public:

    enum {NODE_NULL, NODE_LEAF, NODE_INTERNAL} nodeType;    // Node type
    Params * params;                                        // Access to the problem and dataset parameters
    int splitAttribute;                                        // Attribute to which the split is applied (filled through the greedy algorithm)
    double splitValue;                                            // Threshold value for the split (for numerical attributes the left branch will be <= splitValue, for categorical will be == splitValue)
    std::vector <int> samples;                                // Samples from the training set at this node
    std::vector <int> nbSamplesClass;                        // Number of samples of each class at this node (for each class)
    int nbSamplesNode;                                        // Total number of samples in this node
    int majorityClass;                                        // Majority class in this node
    int maxSameClass;                                        // Maximum number of elements of the same class in this node
    double entropy;                                            // Entropy in this node
    
    void evaluate()
    {
        entropy = 0.0;
        for (int c = 0; c < params->nbClasses; c++)
        {
            if (nbSamplesClass[c] > 0)
            {
                double frac = (double)nbSamplesClass[c]/(double)nbSamplesNode;
                entropy -= frac * log2(frac);
                if (nbSamplesClass[c] > maxSameClass)
                {
                    maxSameClass = nbSamplesClass[c];
                    majorityClass = c;
                }
            }
        }
    }

    void addSample(int i)
    {
        samples.push_back(i);
        nbSamplesClass[params->dataClasses[i]]++;
        nbSamplesNode++;
    }

    DTNode(Params * params):params(params)
    {
        nodeType = NODE_NULL;
        splitAttribute = -1;
        splitValue = -1.e30;
        nbSamplesClass = std::vector<int>(params->nbClasses, 0);
        nbSamplesNode = 0;
        majorityClass = -1;
        maxSameClass = 0;
        entropy = -1.e30;
    }
};

class Solution
{
public:

    // Vector representing the tree
    // Parent of tree[k]: tree[(k-1)/2]
    // Left child of tree[k]: tree[2*k+1]
    // Right child of tree[k]: tree[2*k+2]
    std::vector <DTNode> tree;
    // Access to the problem and dataset parameters
    Params * params;

    // Prints the final solution
    void printAndExport(void)
    {
        int nbMisclassifiedSamples = 0;
        std::cout << std::endl << "---------------------------------------- PRINTING SOLUTION ----------------------------------------" << std::endl;
        for (int d = 0; d <= params->maxDepth; d++)
        {
            // Printing one complete level of the tree
            for (int i = pow(2, d) - 1; i < pow(2, d + 1) - 1; i++)
            {
                if (tree[i].nodeType == DTNode::NODE_INTERNAL)
                    std::cout << "(N" << i << ",A[" << tree[i].splitAttribute << "]" << (params->attributeTypes[tree[i].splitAttribute] == TYPE_NUMERICAL ? "<=" : "=") << tree[i].splitValue << ") ";
                else if (tree[i].nodeType == DTNode::NODE_LEAF)
                {
                    int misclass = tree[i].nbSamplesNode - tree[i].nbSamplesClass[tree[i].majorityClass];
                    nbMisclassifiedSamples += misclass;
                    std::cout << "(L" << i << ",C" << tree[i].majorityClass << "," << tree[i].nbSamplesClass[tree[i].majorityClass] << "," << misclass << ") ";
                }
            }
            std::cout << std::endl;
        }
        std::cout << nbMisclassifiedSamples << "/" << params->nbSamples << " MISCLASSIFIED SAMPLES" << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------" << std::endl << std::endl;
    }

    Solution(Params * params):params(params)
    {
        // Initializing tree data structure and the nodes inside -- The size of the tree is 2^{maxDepth} - 1
        tree = std::vector <DTNode>(pow(2,params->maxDepth+1)-1,DTNode(params));

        // The tree is initially made of a single leaf (the root node)
        tree[0].nodeType = DTNode::NODE_LEAF;
        for (int i = 0; i < params->nbSamples; i++)
            tree[0].addSample(i);
        tree[0].evaluate();
    };
};

class Greedy
{
private:

    Params * params;         // Access to the problem and dataset parameters
    Solution * solution;     // Access to the solution structure to be filled

    // Main recursive function to run the greedy algorithm on the tree
    // Recursive call on a given node and level in the tree
    void recursiveConstruction(int node, int level);

    // Calculates the best split threshold for a continuous attribute
    // Complexity proportional to the number of samples
    double calculateBestSplitContinuous(int atributeIndex, const std::vector<int> & samples);

public:

    // Run the algorithm
    void run();

    // Constructor
    Greedy(Params * params, Solution * solution): params(params), solution(solution){};
};

void Greedy::run()
{
    // Call the recursive procedure on the root node at level 0
    recursiveConstruction(0,0);
}

void Greedy::recursiveConstruction(int node, int level)
{
    /* BASE CASES -- MAXIMUM LEVEL HAS BEEN ATTAINED OR ALL SAMPLES BELONG TO THE SAME CLASS */
    if (level >= params->maxDepth || solution->tree[node].maxSameClass == solution->tree[node].nbSamplesNode)
        return;

    /* LOOK FOR A BEST SPLIT */
    bool allIdentical = true; // To detect contradictory data
    int nbSamplesNode = solution->tree[node].nbSamplesNode;
    double originalEntropy = solution->tree[node].entropy;
    double bestInformationGain = -1.e30;
    int bestSplitAttribute = -1;
    double bestSplitThrehold = -1.e30;
    for (int att = 0; att < params->nbAttributes; att++)
    {
        if (params->attributeTypes[att] == TYPE_NUMERICAL)
        {
            /* CASE 1) -- FIND SPLIT WITH BEST INFORMATION GAIN FOR NUMERICAL ATTRIBUTE c */
             
            // Define some data structures
            std::vector <std::pair<double, int>> orderedSamples;        // Order of the samples according to attribute c
            std::set<double> attributeLevels;                            // Store the possible levels of this attribute among the samples (will allow to "skip" samples with equal attribute value)
            for (int s : solution->tree[node].samples)
            {
                orderedSamples.push_back(std::pair<double, int>(params->dataAttributes[s][att], params->dataClasses[s]));
                attributeLevels.insert(params->dataAttributes[s][att]);
            }
            if (attributeLevels.size() <= 1) continue;                    // If all sample have the same level for this attribute, it's useless to look for a split
            else allIdentical = false;
            std::sort(orderedSamples.begin(), orderedSamples.end());
            
            // Initially all samples are on the right
            std::vector <int> nbSamplesClassLeft = std::vector<int>(params->nbClasses, 0);
            std::vector <int> nbSamplesClassRight = solution->tree[node].nbSamplesClass;
            int indexSample = 0;
            for (double attributeValue : attributeLevels) // Go through all possible attribute values in increasing order
            {
                // Iterate on all samples with this attributeValue and switch them to the left
                while (indexSample < nbSamplesNode && orderedSamples[indexSample].first < attributeValue + MY_EPSILON)
                {
                    nbSamplesClassLeft[orderedSamples[indexSample].second]++;
                    nbSamplesClassRight[orderedSamples[indexSample].second]--;
                    indexSample++;
                }
                
                if (indexSample != nbSamplesNode) // No need to consider the case in which all samples have been switched to the left
                {
                    // Evaluate entropy of the two resulting sample sets
                    double entropyLeft = 0.0;
                    double entropyRight = 0.0;
                    for (int c = 0; c < params->nbClasses; c++)
                    {
                        // Remark that indexSample contains at this stage the number of samples in the left
                        if (nbSamplesClassLeft[c] > 0)
                        {
                            double fracLeft = (double)nbSamplesClassLeft[c] / (double)(indexSample);
                            entropyLeft -= fracLeft * log2(fracLeft);
                        }
                        if (nbSamplesClassRight[c] > 0)
                        {
                            double fracRight = (double)nbSamplesClassRight[c] / (double)(nbSamplesNode - indexSample);
                            entropyRight -= fracRight * log2(fracRight);
                        }
                    }

                    // Evaluate the information gain and store if this is the best option found until now
                    double informationGain = originalEntropy - ((double)indexSample*entropyLeft + (double)(nbSamplesNode - indexSample)*entropyRight) / (double)nbSamplesNode;
                    if (informationGain > bestInformationGain)
                    {
                        bestInformationGain = informationGain;
                        bestSplitAttribute = att;
                        bestSplitThrehold = attributeValue;
                    }
                }
            }
        }
        else
        {
            /* CASE 2) -- FIND BEST SPLIT FOR CATEGORICAL ATTRIBUTE c */

            // Count for each level of attribute c and each class the number of samples
            std::vector <int> nbSamplesLevel = std::vector <int>(params->nbLevels[att],0);
            std::vector <int> nbSamplesClass = std::vector <int>(params->nbClasses, 0);
            std::vector < std::vector <int> > nbSamplesLevelClass = std::vector< std::vector <int> >(params->nbLevels[att], std::vector <int>(params->nbClasses,0));
            for (int s : solution->tree[node].samples)
            {
                nbSamplesLevel[params->dataAttributes[s][att]]++;
                nbSamplesClass[params->dataClasses[s]]++;
                nbSamplesLevelClass[params->dataAttributes[s][att]][params->dataClasses[s]]++;
            }

            // Calculate information gain for a split at each possible level of attribute c
            for (int level = 0; level < params->nbLevels[att]; level++)
            {
                if (nbSamplesLevel[level] > 0 && nbSamplesLevel[level] < nbSamplesNode)
                {
                    // Evaluate entropy of the two resulting sample sets
                    allIdentical = false;
                    double entropyLevel = 0.0;
                    double entropyOthers = 0.0;
                    for (int c = 0; c < params->nbClasses; c++)
                    {
                        if (nbSamplesLevelClass[level][c] > 0)
                        {
                            double fracLevel = (double)nbSamplesLevelClass[level][c] / (double)nbSamplesLevel[level] ;
                            entropyLevel -= fracLevel * log2(fracLevel);
                        }
                        if (nbSamplesClass[c] - nbSamplesLevelClass[level][c] > 0)
                        {
                            double fracOthers = (double)(nbSamplesClass[c] - nbSamplesLevelClass[level][c]) / (double)(nbSamplesNode - nbSamplesLevel[level]);
                            entropyOthers -= fracOthers * log2(fracOthers);
                        }
                    }

                    // Evaluate the information gain and store if this is the best option found until now
                    double informationGain = originalEntropy - ((double)nbSamplesLevel[level] *entropyLevel + (double)(nbSamplesNode - nbSamplesLevel[level])*entropyOthers) / (double)nbSamplesNode;
                    if (informationGain > bestInformationGain)
                    {
                        bestInformationGain = informationGain;
                        bestSplitAttribute = att;
                        bestSplitThrehold = level;
                    }
                }
            }
        }
    }

    /* SPECIAL CASE TO HANDLE POSSIBLE CONTADICTIONS IN THE DATA */
    // (Situations where the same samples have different classes -- In this case no improving split can be found)
    if (allIdentical) return;

    /* APPLY THE SPLIT AND RECURSIVE CALL */
    solution->tree[node].splitAttribute = bestSplitAttribute;
    solution->tree[node].splitValue = bestSplitThrehold;
    solution->tree[node].nodeType = DTNode::NODE_INTERNAL;
    solution->tree[2*node+1].nodeType = DTNode::NODE_LEAF ;
    solution->tree[2*node+2].nodeType = DTNode::NODE_LEAF ;
    for (int s : solution->tree[node].samples)
    {
        if ((params->attributeTypes[bestSplitAttribute] == TYPE_NUMERICAL   && params->dataAttributes[s][bestSplitAttribute] < bestSplitThrehold + MY_EPSILON)||
            (params->attributeTypes[bestSplitAttribute] == TYPE_CATEGORICAL && params->dataAttributes[s][bestSplitAttribute] < bestSplitThrehold + MY_EPSILON && params->dataAttributes[s][bestSplitAttribute] > bestSplitThrehold - MY_EPSILON))
            solution->tree[2*node+1].addSample(s);
        else
            solution->tree[2*node+2].addSample(s);
    }
    solution->tree[2*node+1].evaluate(); // Setting all other data structures
    solution->tree[2*node+2].evaluate(); // Setting all other data structures
    recursiveConstruction(2*node+1,level+1); // Recursive call
    recursiveConstruction(2*node+2,level+1); // Recursive call
}

#endif /* cart_h */
