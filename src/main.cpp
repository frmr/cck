#include <iostream>
#include "cck.h"

using namespace std;

int main()
{
	cck::Globe globe( 0, 6370.0 );
	globe.AddNode( 0, 0.0, 0.0, cck::Globe::NodeType::LAND, 2500.0 );

	if ( globe.AddNode( 1, 0.0, 90.0, cck::Globe::NodeType::LAND, 2500.0 ) == cck::NodeError::SUCCESS )
	{
		cout << "Success" << endl;
	}

	if ( globe.AddLink( cck::Globe::NodeType::LAND, 0, 1 ) == cck::LinkError::SUCCESS )
	{
		cout << "Success" << endl;
	}


	cout << globe.Distance( cck::GeoCoord( 0.0, 0.0 ), cck::GeoCoord( 0.0, 90.0 ) ) << endl;

	return 0;
}
