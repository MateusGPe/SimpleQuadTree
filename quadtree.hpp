/*
Copyright 2017, Mateus Gomes Pereira

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#ifndef QuadTree_Header_impl_
#define QuadTree_Header_impl_
#include <vector>
#include <array>
#include <cassert>
#include <unordered_map>

namespace QuadTree {
    namespace qPrivate {
        template <class qObjectType, class qAlignedBox, class qUint = uint32_t>
        struct NodeStruct {
            NodeStruct( qUint l, qAlignedBox b ): locCode( l ), boundBox( b ),
                hasChild( false ) {}
            std::vector<qObjectType *> objects;
            qUint locCode;
            qAlignedBox boundBox;
            bool hasChild;
        };
    };
    template <class qObjectType, class qAlignedBox, class qUint = uint32_t>
    class HashQuadTree {
    private:
        using NodeType = qPrivate::NodeStruct<qObjectType, qAlignedBox, qUint>;
        uint32_t qMaxDepht;
        uint32_t qMaxObj;
        std::unordered_map<qUint, NodeType> qnodes;

        inline static qUint getChildLoc( qUint loc, uint8_t ch ) {
            assert( loc != 0 );
            return ( loc << 2 ) | ch;
        }
        inline static qUint getParentLoc( qUint loc ) {
            assert( loc != 0 );
            return loc >> 2;
        }

        inline static size_t getNodeDepth( qUint locCode ) {
            assert( locCode != 0 );

#if defined(__GNUC__)
            return ( 31 - __builtin_clz( locCode ) ) / 2;
#elif defined(_MSC_VER)
            long msb;
            _BitScanReverse( &msb, locCode );
            return msb / 2;
#endif
        }
        inline uint8_t getQuad( const qAlignedBox &bb, std::array<qAlignedBox, 4> &aBB );

        void insert__r( NodeType *qthis, qObjectType *obj );
        NodeType *LookupNode( qUint locCode );

        void query__r( NodeType *qn, const qAlignedBox &bx, std::vector<qObjectType *> &res );
        bool query__r2( NodeType *qn, const qAlignedBox &bx, std::vector<qObjectType *> &res );
    public:
        HashQuadTree( const qAlignedBox &aabb, uint32_t qMo = 8, uint32_t qMd = 5 );
        void insert( qObjectType *obj );
        std::vector<qObjectType *> query( const qAlignedBox &bx, std::vector<qObjectType *> &res );

        void draw__r( NodeType *qn, void ( *drw )( const qAlignedBox &, uint32_t, bool ) );

        void draw( void ( *drw )( const qAlignedBox &, uint32_t, bool ) );
    };
}
#endif
#ifdef QUADTREE_IMPLEMENTATION
namespace QuadTree{
#define decl_HQT(name) HashQuadTree<qObjectType, qAlignedBox, qUint>::name
#define tmlp_HQT template <class qObjectType, class qAlignedBox, class qUint>
    tmlp_HQT
    using NodeType = typename qPrivate::NodeStruct<qObjectType, qAlignedBox, qUint>;
    tmlp_HQT
    inline uint8_t decl_HQT( getQuad )( const qAlignedBox &bb, std::array<qAlignedBox, 4> &aBB
                                      ) {
        uint8_t quad;

        quad = aBB[2].Contains( bb );
        quad = ( quad << 1 ) | quad;
        quad |= aBB[0].Contains( bb );
        quad |= aBB[1].Contains( bb ) << 1;
        quad |= aBB[3].Contains( bb ) << 2;
        return quad;
    }
    tmlp_HQT
    void decl_HQT( insert__r )( NodeType *qthis, qObjectType *obj ) {
        assert( qthis != nullptr );
        std::array<qAlignedBox, 4> aBB;

        qthis->boundBox.subDivide( aBB );
        uint8_t quad = getQuad( obj->getAABB(), aBB );

        if( qthis->hasChild && ( quad != 0 ) ) {
            insert__r( LookupNode( this->getChildLoc( qthis->locCode, quad - 1 ) ), obj );
            return;
        }

        qthis->objects.push_back( obj );

        if( ( qthis->objects.size() <= qMaxObj ) || ( this->getNodeDepth( qthis->locCode ) > qMaxDepht ) )
            return;

        uint32_t loc;
        loc = this->getChildLoc( qthis->locCode, 0 );
        qnodes.insert( {loc, NodeType( loc, aBB[0] )} );
        loc = this->getChildLoc( qthis->locCode, 1 );
        qnodes.insert( {loc, NodeType( loc, aBB[1] )} );
        loc = this->getChildLoc( qthis->locCode, 2 );
        qnodes.insert( {loc, NodeType( loc , aBB[2] )} );
        loc = this->getChildLoc( qthis->locCode, 3 );
        qnodes.insert( {loc, NodeType( loc, aBB[3] )} );
        qthis->hasChild = true;

        std::vector<qObjectType *> sw;

        for( size_t i = 0; i < qthis->objects.size(); i++ ) {
            quad =  getQuad( qthis->objects[i]->getAABB(), aBB );

            if( quad != 0 ) {
                quad--;
                NodeType *qother = LookupNode( this->getChildLoc( qthis->locCode, quad ) );

                if( qother == nullptr ) {
                    std::cerr << "Look for node " << ( uint16_t )quad << "failed... " << std::endl;
                    sw.push_back( qthis->objects[i] );
                    continue;
                }

                qother->objects.push_back( qthis->objects[i] );
            } else {
                sw.push_back( qthis->objects[i] );
            }
        }

        qthis->objects = std::move( sw );
    }
    tmlp_HQT
    NodeType<qObjectType, qAlignedBox, qUint> *decl_HQT( LookupNode )( qUint locCode ) {
        const auto iter = qnodes.find( locCode );
        return ( iter == qnodes.end() ? nullptr : & ( iter->second ) );
    }
    tmlp_HQT
    void decl_HQT( query__r )( NodeType *qn, const qAlignedBox &bx,
                               std::vector<qObjectType *> &res ) {
        if( qn == nullptr )
            return;

        if( !qn->boundBox.Overlaps( bx ) )
            return;

        for( auto i : qn->objects ) {
            res.push_back( i );
        }

        query__r( LookupNode( this->getChildLoc( qn->locCode, 0 ) ), bx, res );
        query__r( LookupNode( this->getChildLoc( qn->locCode, 1 ) ), bx, res );
        query__r( LookupNode( this->getChildLoc( qn->locCode, 2 ) ), bx, res );
        query__r( LookupNode( this->getChildLoc( qn->locCode, 3 ) ), bx, res );
    }
    tmlp_HQT
    bool decl_HQT( query__r2 )( NodeType *qn, const qAlignedBox
                                &bx, std::vector<qObjectType *> &res ) {
        if( qn == nullptr )
            return false;

        if( !qn->boundBox.Overlaps( bx ) )
            return false;

        if( !( query__r2( LookupNode( this->getChildLoc( qn->locCode, 0 ) ), bx, res ) ||
                query__r2( LookupNode( this->getChildLoc( qn->locCode, 1 ) ), bx, res ) ||
                query__r2( LookupNode( this->getChildLoc( qn->locCode, 2 ) ), bx, res ) ||
                query__r2( LookupNode( this->getChildLoc( qn->locCode, 3 ) ), bx, res ) ) )
            for( auto i : qn->objects ) {
                res.push_back( i );
            }

        return qn->boundBox.Contains( bx );
    }
    tmlp_HQT
    decl_HQT( HashQuadTree )( const qAlignedBox &aabb, uint32_t qMo, uint32_t qMd ):
        qMaxDepht( qMd ), qMaxObj( qMo ) {
        NodeType root( 1, aabb );
        qnodes.insert( {1, root} );
    }
    tmlp_HQT
    void decl_HQT( insert )( qObjectType *obj ) {
        insert__r( LookupNode( 1 ), obj );
    }
    tmlp_HQT
    std::vector<qObjectType *> decl_HQT( query )( const qAlignedBox &bx, std::vector<qObjectType *>
            &res ) {
        query__r( LookupNode( 1 ), bx, res );
        return res;
    }
    tmlp_HQT
    void decl_HQT( draw__r )( NodeType *qn, void ( *drw )(
                                  const qAlignedBox &, uint32_t, bool ) ) {
        if( qn == nullptr )
            return;

        drw( qn->boundBox, qn->locCode, false );

        for( auto i : qn->objects ) {
            drw( i->getAABB(), qn->locCode, true );
        }

        draw__r( LookupNode( this->getChildLoc( qn->locCode, 0 ) ), drw );
        draw__r( LookupNode( this->getChildLoc( qn->locCode, 1 ) ), drw );
        draw__r( LookupNode( this->getChildLoc( qn->locCode, 2 ) ), drw );
        draw__r( LookupNode( this->getChildLoc( qn->locCode, 3 ) ), drw );
    }
    tmlp_HQT
    void decl_HQT( draw )( void ( *drw )( const qAlignedBox &, uint32_t, bool ) ) {
        draw__r( LookupNode( 1 ), drw );
    }
#undef decl_HQT
#undef tmlp_HQT
}
#endif
