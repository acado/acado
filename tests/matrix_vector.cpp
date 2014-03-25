#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MatrixVectorTests
#include <boost/test/unit_test.hpp>

#include <acado/matrix_vector/matrix_vector.hpp>

USING_NAMESPACE_ACADO

using namespace std;

BOOST_AUTO_TEST_CASE( vector_ctors )
{
	double cc[ 3 ] = {1, 2};
	vector< double > dd( 2 ); dd[ 0 ] = -10; dd[ 1 ] = 99;
	DVector a, b( 5 ), c(2, cc), d( dd );

    BOOST_REQUIRE( a.getDim() == 0 );
    BOOST_REQUIRE( b.getDim() == 5 );
    BOOST_REQUIRE( acadoIsEqual(b( 0 ), 0) );
    BOOST_REQUIRE( c.getDim() == 2 );
    BOOST_REQUIRE( acadoIsEqual(c( 1 ), 2) );
    BOOST_REQUIRE( d.getDim() == 2 );
    BOOST_REQUIRE( acadoIsEqual(d( 0 ), -10) && acadoIsEqual(d( 1 ), 99) );
}
