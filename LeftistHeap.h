#pragma once
#ifndef BINARY_HEAP_H_
#define BINARY_HEAP_H_
#include <iostream>
#include <list>
// Leftist heap class
//
// CONSTRUCTION: with no parameters
//
// ******************PUBLIC OPERATIONS*********************
// void insert( x )       --> Insert x
// deleteMin( minItem )   --> Remove (and optionally return) smallest item
// Comparable findMin( )  --> Return smallest item
// bool isEmpty( )        --> Return true if empty; else false
// bool isFull( )         --> Return true if full; else false
// void makeEmpty( )      --> Remove all items
// void merge( rhs )      --> Absorb rhs into this heap
// ******************ERRORS********************************
// Throws Underflow and Overflow as warranted
  // Node and forward declaration because g++ does
  // not understand nested classes.
template<class Comparable>
struct DefaultComparator
{
    static int Compare(const Comparable &first, const Comparable &second)
    {
        if (first > second)
            return +1;
        else if(first < second)
            return -1;
        else
            return 0;
    }
};

template <typename Comparable,typename Comparator>
class LeftistHeap;

template <class Comparable>
class LeftistNode
{
public:
	Comparable   element;
    LeftistNode *left;
    LeftistNode *right;
    int          npl;
    LeftistNode( const Comparable & theElement, LeftistNode *lt = NULL,
                 LeftistNode *rt = NULL, int np = 0 )
      : element( theElement ), left( lt ), right( rt ), npl( np ) { }
    //friend class LeftistHeap<Comparable>;
};
template <class Comparable,class Comparator = DefaultComparator<Comparable> >
class LeftistHeap
{
public:
  LeftistHeap( );
  LeftistHeap( const LeftistHeap<Comparable, Comparator> & rhs );
  ~LeftistHeap( );
private:
  bool isEmpty( ) const;
  const Comparable *findMin( ) const;
  bool popMin(Comparable &min);
  void insert( const Comparable & x );
  void deleteMin( );
  void makeEmpty( );
  void merge( LeftistHeap & rhs );
  const LeftistHeap & operator=( const LeftistHeap & rhs );
	void InorderConstruct(const std::list<Comparable> &lstInorder);
public:
	void push_back(const Comparable& e)
	{
		m_sz ++;
    insert(e);
	}
  void push_front(const Comparable& e)
  {
    m_sz ++;
    insert(e);
  }
	bool empty() const
	{
		return isEmpty();
	}
	Comparable front() const
	{
		return *findMin();
	}
	void pop_front()
	{
		m_sz --;
    deleteMin();
	}

  int size() const
  {
    return m_sz;
  }

private:
    LeftistNode<Comparable> *root;
    LeftistNode<Comparable> * merge( LeftistNode<Comparable> *h1,
                                     LeftistNode<Comparable> *h2 ) const;
    LeftistNode<Comparable> * merge1( LeftistNode<Comparable> *h1,
                                      LeftistNode<Comparable> *h2 ) const;
    void swapChildren( LeftistNode<Comparable> * t ) const;
    void reclaimMemory( LeftistNode<Comparable> * t ) const;
    LeftistNode<Comparable> * clone( LeftistNode<Comparable> *t ) const;
    int m_sz;
};


/**
 * Construct the leftist heap.
 */
template <class Comparable,class Comparator>
LeftistHeap<Comparable, Comparator>::LeftistHeap( ) : m_sz(0)
{
    root = NULL;
}
/**
 * Copy constructor.
 */
template <class Comparable,class Comparator>
LeftistHeap<Comparable,Comparator>::LeftistHeap( const LeftistHeap<Comparable,Comparator> & rhs ) : m_sz(0)
{
    root = NULL;
    *this = rhs;
}
/**
 * Destruct the leftist heap.
 */
template <class Comparable,class Comparator>
LeftistHeap<Comparable,Comparator>::~LeftistHeap( )
{
    makeEmpty( );
}
/**
 * Merge rhs into the priority queue.
 * rhs becomes empty. rhs must be different from this.
 */
template <class Comparable,class Comparator>
void LeftistHeap<Comparable,Comparator>::merge( LeftistHeap & rhs )
{
    if( this == &rhs )    // Avoid aliasing problems
        return;
    root = merge( root, rhs.root );
    rhs.root = NULL;
}
/**
 * Internal method to merge two roots.
 * Deals with deviant cases and calls recursive merge1.
 */
template <class Comparable, class Comparator>
LeftistNode<Comparable> *
LeftistHeap<Comparable, Comparator>::merge( LeftistNode<Comparable> * h1,
                                LeftistNode<Comparable> * h2 ) const
{
    if( h1 == NULL )
        return h2;
    if( h2 == NULL )
        return h1;
    //if( h1->element < h2->element )
    if (Comparator::Compare(h1->element, h2->element) < 0)
        return merge1( h1, h2 );
    else
        return merge1( h2, h1 );
}
/**
 * Internal method to merge two roots.
 * Assumes trees are not empty, and h1's root contains smallest item.
 */
template <class Comparable,class Comparator>
LeftistNode<Comparable> *
LeftistHeap<Comparable,Comparator>::merge1( LeftistNode<Comparable> * h1,
                                 LeftistNode<Comparable> * h2 ) const
{
    if( h1->left == NULL )   // Single node
        h1->left = h2;       // Other fields in h1 already accurate
    else
    {
        h1->right = merge( h1->right, h2 );
        if( h1->left->npl < h1->right->npl )
            swapChildren( h1 );
        h1->npl = h1->right->npl + 1;
    }
    return h1;
}
/**
 * Swaps t's two children.
 */
template <class Comparable,class Comparator>
void LeftistHeap<Comparable,Comparator>::swapChildren( LeftistNode<Comparable> * t ) const
{
    LeftistNode<Comparable> *tmp = t->left;
    t->left = t->right;
    t->right = tmp;
}
/**
 * Insert item x into the priority queue, maintaining heap order.
 */
template <class Comparable,class Comparator>
void LeftistHeap<Comparable,Comparator>::insert( const Comparable & x )
{
    root = merge( new LeftistNode<Comparable>( x ), root );
}
/**
 * Find the smallest item in the priority queue.
 * Return the smallest item, or throw Underflow if empty.
 */
template <class Comparable,class Comparator>
const Comparable* LeftistHeap<Comparable,Comparator>::findMin( ) const
{
    if( isEmpty( ) )
        return NULL;
    else
        return &root->element;
}

template<class Comparable, class Comparator>
bool LeftistHeap<Comparable,Comparator>::popMin(Comparable &min)
{
  if (NULL == root)
    return false;
  else
  {
    min = root->element;
    deleteMin();
    return true;
  }
}


/**
 * Remove the smallest item from the priority queue.
 * Throws Underflow if empty.
 */
template <class Comparable,class Comparator>
void LeftistHeap<Comparable,Comparator>::deleteMin( )
{
    if( !isEmpty( ) )
    {
        LeftistNode<Comparable> *oldRoot = root;
        root = merge( root->left, root->right );
        delete oldRoot;
    }
}

/**
 * Test if the priority queue is logically empty.
 * Returns true if empty, false otherwise.
 */
template <class Comparable,class Comparator>
bool LeftistHeap<Comparable,Comparator>::isEmpty( ) const
{
    return root == NULL;
}

/**
 * Make the priority queue logically empty.
 */
template <class Comparable,class Comparator>
void LeftistHeap<Comparable,Comparator>::makeEmpty( )
{
    reclaimMemory( root );
    root = NULL;
}
/**
 * Deep copy.
 */
template <class Comparable,class Comparator>
const LeftistHeap<Comparable,Comparator> &
LeftistHeap<Comparable,Comparator>::
operator=( const LeftistHeap<Comparable,Comparator> & rhs )
{
    if( this != &rhs )
    {
        makeEmpty( );
        root = clone( rhs.root );
    }
    return *this;
}
/**
 * Internal method to make the tree empty.
 * WARNING: This is prone to running out of stack space;
 *          exercises suggest a solution.
 */
template <class Comparable,class Comparator>
void LeftistHeap<Comparable,Comparator>::reclaimMemory( LeftistNode<Comparable> * t ) const
{
    if( t != NULL )
    {
        reclaimMemory( t->left );
        reclaimMemory( t->right );
        delete t;
    }
}
/**
 * Internal method to clone subtree.
 * WARNING: This is prone to running out of stack space.
 *          exercises suggest a solution.
 */
template <class Comparable,class Comparator>
LeftistNode<Comparable> *
LeftistHeap<Comparable,Comparator>::clone( LeftistNode<Comparable> * t ) const
{
    if( t == NULL )
        return NULL;
    else
        return new LeftistNode<Comparable>( t->element, clone( t->left ),
                                      clone( t->right ), t->npl );
}


template<class Comparable, class Comparator>
void LeftistHeap<Comparable,Comparator>::InorderConstruct(const std::list<Comparable> &lstInorder)
{
    ASSERT(isEmpty());
    if (!lstInorder.empty())
    {
        typename std::list<Comparable>::const_iterator it = lstInorder.end();
        root = new LeftistNode<Comparable>(*(--it));
        root->npl = 1;
        while(it != lstInorder.begin())
        {
            LeftistNode<Comparable> *node = new LeftistNode<Comparable>(*(--it));
            node->left = root;
            node->npl = 1;
            root = node;
        }
    }
}

#endif
