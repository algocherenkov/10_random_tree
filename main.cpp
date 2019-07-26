#include <fstream>
#include <math.h>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <ctime>

#define BOOST_TEST_MODULE test_main

#include <boost/test/included/unit_test.hpp>

using namespace boost::unit_test;
BOOST_AUTO_TEST_SUITE(test_suite_main)

template <typename KeyType, typename ValueType>
class Node
{
public:
    Node(Node* node):
        key_(node->key_),
        value_(node->value_)
    {}

    Node(KeyType key, ValueType value):
        key_(key),
        value_(value)
    {}

    const Node& operator=(const Node& node)
    {
        leftSubTree_     = node.leftSubTree_;
        rightSubTree_    = node.rightSubTree_;
        value_          = node.value_;
        key_            = node.key_;
        height_          = node.height_;
        return *this;
    }

private:
    int getHeight() const { return height_; }

    int getHeightLeft(Node* node) const { return node->leftSubTree_? node->leftSubTree_->getHeight():0;}
    int getHeightRight(Node* node) const { return node->rightSubTree_? node->rightSubTree_->getHeight():0;}

    int getBalance(Node* node) const { return node? getHeightLeft(node) - getHeightRight(node): 0; }
    void recalculateHeight(Node* node) { node->height_ = std::max(getHeightLeft(node), getHeightRight(node)) + 1; }

    Node* rotateLeft(Node* pivotNode)
    {
        if(!pivotNode)
            return nullptr;

        auto rightSubtree = pivotNode->rightSubTree_;
        if(!rightSubtree)
            return nullptr;

        pivotNode->rightSubTree_ = rightSubtree->leftSubTree_;
        rightSubtree->leftSubTree_ = pivotNode;

        recalculateHeight(pivotNode);
        recalculateHeight(rightSubtree);

        return rightSubtree;
    }

    Node* rotateRight(Node* pivotNode)
    {
        if(!pivotNode)
            return nullptr;

        auto leftSubtree = pivotNode->leftSubTree_;
        if(!leftSubtree)
            return nullptr;

        pivotNode->leftSubTree_= leftSubtree->rightSubTree_;
        leftSubtree->rightSubTree_ = pivotNode;

        recalculateHeight(pivotNode);
        recalculateHeight(leftSubtree);

        return leftSubtree;
    }

    Node* join(Node* first, Node* second)
    {
        if(!first)
            return second;

        if(!second)
            return first;

        std::srand(static_cast<unsigned int>(std::time(0)));
        if( std::rand() % (first->height_ + second->height_) < first->height_)
        {
            first->rightSubTree_= join(first->rightSubTree_, second);
            recalculateHeight(first);
            return first;
        }
        else
        {
            second->leftSubTree_ = join(first, second->leftSubTree_);
            recalculateHeight(second);
            return second;
        }
    }

    Node* insertToRoot(Node* node, KeyType key, ValueType value)
    {
        if(!node)
            return new Node(key, value);

        if(key <= node->key_)
        {
            node->leftSubTree_ = insertToRoot(node->leftSubTree_, key, value);
            return rotateRight(node);
        }
        else
        {
            node->rightSubTree_ = insertToRoot(node->rightSubTree_, key, value);
            return rotateLeft(node);
        }
    }

public:
    /*basic interface*/
    Node* insert(Node* node, KeyType key, ValueType value)
    {
        if(!node)
            return new Node(key, value);

        std::srand(static_cast<unsigned int>(time(0)));
        if(std::rand() % (node->height_ + 1) == 0)
            return insertToRoot(node, key, value);

        if(key <= node->key_)
            node->leftSubTree_ = insert(node->leftSubTree_, key, value);
        else
            node->rightSubTree_ = insert(node->rightSubTree_, key, value);

        recalculateHeight(node);
        return node;
    }

    Node* remove(Node* node, KeyType key)
    {
        if(!node)
            return node;
        if(node->key_ == key)
        {
            Node* result = join(node->leftSubTree_, node->rightSubTree_);
            delete node;
            return result;
        }
        else if(key < node->key_)
            node->leftSubTree_ = remove(node->leftSubTree_, key);
        else
            node->rightSubTree_ = remove(node->rightSubTree_, key);
        return node;
    }

    Node* find(Node* node, KeyType key)
    {
        if(!node)
            return nullptr;
        if(key == node->getKey() )
            return node;
        if(key < node->getKey())
            return find(node->leftSubTree_, key);
        else
            return find(node->rightSubTree_, key);
    }

    /*helpers*/
    Node* getLeftSubTree() const { return leftSubTree_; }
    Node* getRightSubTree() const { return rightSubTree_; }
    KeyType getKey() const { return key_; }

private:
    Node*       leftSubTree_ {nullptr};
    Node*       rightSubTree_ {nullptr};
    ValueType   value_;
    KeyType     key_;
    int         height_ {1};
};

template<typename KeyType, typename ValueType>
void fillVectorFromTree(std::vector<int>& vector, Node<KeyType, ValueType>* root)
{
    if(root)
    {
        if(root->getLeftSubTree())
            fillVectorFromTree(vector, root->getLeftSubTree());

        vector.push_back(root->getKey());

        if(root->getRightSubTree())
            fillVectorFromTree(vector, root->getRightSubTree());
    }
}

BOOST_AUTO_TEST_CASE(quicksort_test)
{

    Node<int, std::string>* root = new Node<int, std::string>(5, "first");
    root = root->insert(root, 7, "alala");
    root = root->insert(root, 3, "ololo");
    root = root->insert(root, 10, "atata");
    root = root->insert(root, 1, "azaza");
    root = root->insert(root, 12, "ululu");
    root = root->insert(root, 15, "ahaha");
    root = root->insert(root, 19, "tilitili");
    root = root->insert(root, 4, "bububu");
    root = root->insert(root, -1, "gagaga");
    root = root->insert(root, -3, "fififi");
    root = root->insert(root, 21, "tratata");
    root = root->insert(root, 20, "wiwiwi");

    std::vector<int> keyOrder;
    fillVectorFromTree(keyOrder, root);

    int prev = keyOrder[0];
    for(const auto& key: keyOrder)
    {
        BOOST_CHECK_MESSAGE(prev <= key, "tree after inserting was formed wrong - key: " << key);
        prev = key;
    }

    auto result = root->find(root, 21)->getKey();
    BOOST_CHECK_MESSAGE(result == 21, "find()-method returned wrong key: " << result);
    result = root->find(root, -3)->getKey();
    BOOST_CHECK_MESSAGE(result == -3, "find()-method returned wrong key: " << result);

    root = root->remove(root, 12);
    root = root->remove(root, -1);
    root = root->remove(root, 21);
    root = root->remove(root, 10);
    root = root->remove(root, 5);
    keyOrder.clear();

    fillVectorFromTree(keyOrder, root);

    prev = keyOrder[0];
    for(const auto& key: keyOrder)
    {
        BOOST_CHECK_MESSAGE(prev <= key, "tree after deleting was formed wrong - key: " << key);
        prev = key;
    }

    BOOST_TEST_MESSAGE("end of test");
}

BOOST_AUTO_TEST_SUITE_END()

