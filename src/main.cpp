#include <iostream>
#include "cck.h"

using namespace std;

int main()
{
	cck::Globe globe( 6370.0, 0 );
	globe.AddNode( 0, 0.0, 0.0, 2500.0 );

	if ( globe.AddNode( 1, 0.0, 90.0, 500.0 ) == cck::NodeError::SUCCESS )
	{
		cout << "Success" << endl;
	}

	if ( globe.LinkNodes( 0, 1, 1.0 ) == cck::LinkError::SUCCESS )
	{
		cout << "Success" << endl;
	}

	cout << globe.GetNodeId( 0.0, 45.1 ) << endl;

	return 0;
}
