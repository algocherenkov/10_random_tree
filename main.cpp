#include <fstream>
#include <math.h>
#include <algorithm>
#include <random>

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
        leftSubTree = node.leftSubTree;
        rightSubTree = node.rightSubTree;
        value_ = node.value_;
        key_ = node.key_;
        height = node.height;
        return *this;
    }

    Node* getLeftSubTree() const {return leftSubTree; }
    Node* getRightSubTree() const {return rightSubTree; }
    KeyType getKey() const { return key_; }

private:
    Node*       leftSubTree {nullptr};
    Node*       rightSubTree {nullptr};
    ValueType   value_;
    KeyType     key_;
    int         height{1};

private:

    int getHeight() const { return height; }

    int getHeightLeft(Node* node) const { return node->leftSubTree? node->leftSubTree->getHeight():0;}
    int getHeightRight(Node* node) const { return node->rightSubTree? node->rightSubTree->getHeight():0;}

    int getBalance(Node* node) const { return node? getHeightLeft(node) - getHeightRight(node): 0; }
    void recalculateHeight(Node* node) { node->height = std::max(getHeightLeft(node), getHeightRight(node)) + 1; }

    Node* rotateLeft(Node* pivotNode)
    {
        if(!pivotNode)
            return nullptr;

        auto rightSubtree = pivotNode->rightSubTree;
        if(!rightSubtree)
            return nullptr;

        pivotNode->rightSubTree = rightSubtree->leftSubTree;
        rightSubtree->leftSubTree = pivotNode;

        recalculateHeight(pivotNode);
        recalculateHeight(rightSubtree);

        return rightSubtree;
    }

    Node* rotateRight(Node* pivotNode)
    {
        if(!pivotNode)
            return nullptr;

        auto leftSubtree = pivotNode->leftSubTree;
        if(!leftSubtree)
            return nullptr;

        pivotNode->leftSubTree= leftSubtree->rightSubTree;
        leftSubtree->rightSubTree = pivotNode;

        recalculateHeight(pivotNode);
        recalculateHeight(leftSubtree);

        return leftSubtree;
    }

    Node* bigRotateLeft(Node* node)
    {
        node->rightSubTree = rotateRight(node->rightSubTree);
        return rotateLeft(node);
    }

    Node* bigRotateRight(Node* node)
    {
        node->leftSubTree = rotateLeft(node->leftSubTree);
        return rotateRight(node);
    }

    Node* rebalance(Node* node, KeyType key)
    {
        recalculateHeight(node);
        auto balance = getBalance(node);

        if (balance > 1 && node->leftSubTree && key <= node->leftSubTree->key_)
            return rotateRight(node);

        if (balance < -1 && node->rightSubTree && key > node->rightSubTree->key_)
            return rotateLeft(node);

        if (balance > 1 && node->leftSubTree && key > node->leftSubTree->key_)
            return bigRotateRight(node);

        if (balance < -1 && node->rightSubTree && key < node->rightSubTree->key_)
            return bigRotateLeft(node);

        return node;
    }

    Node* removeNode(KeyType key, Node* node)
    {
        if(node == nullptr)
            return node;

        if(key < node->key_)
            node->leftSubTree = removeNode(key, node->leftSubTree);
        else if(key > node->key_)
            node->rightSubTree = removeNode(key, node->rightSubTree);
        else
        {
            if(node->leftSubTree == nullptr || node->rightSubTree == nullptr)
            {
                Node* temp = node->leftSubTree ? node->leftSubTree: node->rightSubTree;

                if (temp == nullptr)
                {
                    temp = node;
                    node = nullptr;
                }
                else
                {
                    *node = *temp;
                }

                delete temp;
            }
            else
            {
                Node* temp = getMinFromRightSubTree(node->rightSubTree);

                node->key_ = temp->key_;
                node->value_ = temp->value_;

                node->rightSubTree = removeNode(temp->key_, node->rightSubTree);
            }
        }

        if(node == nullptr)
            return node;

        return rebalance(node, key);
    }

    Node* getMinFromRightSubTree(Node* node)
    {
        Node* currentNode = node;

        while(currentNode->leftSubTree != nullptr)
            currentNode = currentNode->leftSubTree;

        return currentNode;
    }

    Node* insertToRoot(KeyType key, ValueType value)
    {

    }

    void regularInsert(KeyType key, ValueType value)
    {
        if(key <= key_)
        {
            if(leftSubTree)
            {
               leftSubTree = leftSubTree->insert(key, value);
            }
            else
            {
                leftSubTree = new Node(key, value);
            }
        }
        else
        {
            if(rightSubTree)
            {
                rightSubTree = rightSubTree->insert(key, value);
            }
            else
            {
                rightSubTree = new Node(key, value);
            }
        }
    }

public:
    Node* insert(KeyType key, ValueType value)
    {
        Node* node;
        std::srand(time(0));
        if(std::rand() % (height + 1) == 0)
            node = insertToRoot(key, value);

        regularInsert(key, value);
        recalculateHeight(this);
        return node? node: this;
    }

    void removeNode(KeyType key)
    {
        removeNode(key, this);
    }


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

    Node<int, std::string> tree(5, "root");
    tree.insert(7, "alala");
    tree.insert(3, "ololo");
    tree.insert(10, "atata");
    tree.insert(1, "azaza");
    tree.insert(12, "ululu");
    tree.insert(15, "ahaha");
    tree.insert(19, "tilitili");
    tree.insert(4, "bububu");
    tree.insert(-1, "gagaga");
    tree.insert(-3, "fififi");
    tree.insert(21, "tratata");
    tree.insert(20, "wiwiwi");

    std::vector<int> keyOrder;
    fillVectorFromTree(keyOrder, &tree);

    int prev = keyOrder[0];
    for(const auto& key: keyOrder)
    {
        BOOST_CHECK_MESSAGE(prev <= key, "tree after inserting was formed wrong - key: " << key);
        prev = key;
    }

    tree.removeNode(12);
    tree.removeNode(-1);
    tree.removeNode(21);
    tree.removeNode(10);
    tree.removeNode(5);
    keyOrder.clear();

    fillVectorFromTree(keyOrder, &tree);

    prev = keyOrder[0];
    for(const auto& key: keyOrder)
    {
        BOOST_CHECK_MESSAGE(prev <= key, "tree after deleting was formed wrong - key: " << key);
        prev = key;
    }

    BOOST_TEST_MESSAGE("end of test");
}

BOOST_AUTO_TEST_SUITE_END()

