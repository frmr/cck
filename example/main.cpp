#include <iostream>
#include "EasyBMP/EasyBMP.h"
#include "../src/cck.h"
#include <cstdlib>
#include <ctime>
#include <limits>
#include <string>

using namespace std;

int main()
{
    int xRes = 512;
    int yRes = 256;

    int seed = time(NULL);
    srand(seed);

    int globeSeed = rand();
    cout << globeSeed << endl;
    cck::Globe globe( 6370.0, globeSeed );
    globe.SetNoiseParameters( 8, 0.75, 0.00015 );

    globe.AddNode( 0,   52.0,   -4.0,   -0.2,   0.5,    600.0 );    //Britain
    globe.AddNode( 1,   46.0,   2.0,    -0.2,   0.5,    600.0 );    //France
    globe.AddNode( 2,   40.0,   -4.0,   -0.2,   1.0,    750.0 );    //Iberia
    globe.AddNode( 3,   50.0,   10.0,   -0.3,   0.75,   500.0 );    //Germany
    globe.AddNode( 4,   42.0,   15.0,   -0.3,   0.75,   550.0 );    //Italy
    globe.AddNode( 5,   65.0,   17.0,   -1.0,   1.5,    1000.0 );   //Scandinavia
    globe.AddNode( 6,   50.0,   25.0,   -0.2,   0.5,    800.0 );    //Poland
    globe.AddNode( 7,   40.0,   25.0,   -0.5,   1.5,    600.0 );    //Balkans
    globe.AddNode( 8,   57.0,   44.0,   -0.3,   0.75,   1200.0 );   //Russia
    globe.AddNode( 9,   38.0,   38.0,   -1.5,   3.0,    1000.0 );   //Turkey
    globe.AddNode( 10,  26.0,   44.0,   -0.5,   1.0,    1600.0 );   //Arabia
    globe.AddNode( 11,  35.0,   56.0,   -1.0,   2.0,    700.0 );    //Persia
    globe.AddNode( 12,  48.0,   68.0,   -0.2,   0.75,   1100.0 );   //Central Asia
    globe.AddNode( 13,  65.0,   75.0,   -0.2,   0.5,    1300.0 );   //Siberia 1
    globe.AddNode( 14,  65.0,   106.0,  -0.3,   0.75,   1400.0 );   //Siberia 2
    globe.AddNode( 15,  32.0,   88.0,   1.0,    4.0,    500.0 );    //Himalayas
    globe.AddNode( 29,  44.0,   95.0,   -1.0,   2.5,    800.0 );    //Xinjiang
    globe.AddNode( 16,  25.0,   80.0,   -0.4,   1.0,    1000.0 );   //North India
    globe.AddNode( 17,  15.0,   77.0,   -0.4,   1.0,    1000.0 );   //South India
    globe.AddNode( 18,  25.0,   102.0,  -1.0,   2.0,    700.0 );    //Burma
    globe.AddNode( 19,  14.0,   103.0,  -0.5,   0.75,   800.0 );    //Indochina
    globe.AddNode( 20,  0.0,    103.0,  -0.4,   1.0,    850.0 );    //Malaysia
    globe.AddNode( 21,  30.0,   107.0,  -0.5,   2.0,    400.0 );    //Inner China
    globe.AddNode( 22,  23.0,   115.0,  -1.0,   1.5,    600.0 );    //South China
    globe.AddNode( 23,  35.0,   115.0,  -0.3,   0.5,    400.0 );    //North China
    globe.AddNode( 24,  47.0,   109.0,  -0.5,   1.5,    900.0 );    //Mongolia
    globe.AddNode( 25,  38.0,   127.0,  -0.5,   0.75,   350.0 );    //Korea
    globe.AddNode( 26,  65.0,   144.0,  -0.5,   1.0,    800.0 );    //Siberia 3
    globe.AddNode( 27,  62.0,   165.0,  -1.0,   2.0,    1000.0 );   //Siberia 4
    globe.AddNode( 28,  36.0,   138.0,  -1.0,   1.5,    800.0 );    //Japan



    globe.LinkNodes( 1,     2,  -1.6,   4.0,    150.0,  50.0 );     //France, Iberia
    globe.LinkNodes( 1,     3,  -0.3,   0.75,   150.0,  50.0 );     //France, Germany
    globe.LinkNodes( 1,     4,  -0.3,   5.0,    150.0,  50.0 );     //France, Italy
    globe.LinkNodes( 4,     2,  -0.3,   0.75,   150.0,  50.0 );     //Italy, Iberia
    globe.LinkNodes( 4,     7,  -0.5,   0.75,   150.0,  50.0 );     //Italy, Balkans
    globe.LinkNodes( 3,     4,  -0.3,   5.0,    150.0,  50.0 );     //Germany, Italy
    globe.LinkNodes( 3,     5,  -0.75,  0.75,   150.0,  50.0 );     //Germany, Scandinavia
    globe.LinkNodes( 3,     6,  -0.4,   0.5,    150.0,  50.0 );     //Germany, Poland
    globe.LinkNodes( 3,     7,  -0.75,  1.5,    150.0,  50.0 );     //Germany, Balkans
    globe.LinkNodes( 5,     8,  -1.0,   1.5,    150.0,  50.0 );     //Scandinavia, Russia
    globe.LinkNodes( 5,     6,  -1.0,   1.5,    150.0,  50.0 );     //Scandinavia, Poland
    globe.LinkNodes( 6,     8,  -0.5,   0.5,    150.0,  50.0 );     //Poland, Russia
    globe.LinkNodes( 6,     7,  -1.0,   1.0,    150.0,  50.0 );     //Poland, Balkans
    globe.LinkNodes( 7,     9,  -3.0,   3.0,    150.0,  50.0 );     //Balkans, Turkey
    globe.LinkNodes( 8,     9,  -1.5,   4.0,    150.0,  50.0 );     //Russia, Turkey
    globe.LinkNodes( 7,     8,  -1.0,   3.0,    150.0,  50.0 );     //Balkans, Russia
    globe.LinkNodes( 9,     10, -3.0,   3.0,    150.0,  50.0 );     //Turkey, Arabia
    globe.LinkNodes( 8,     12, -0.5,   0.75,   150.0,  50.0 );     //Russia, Central Asia
    globe.LinkNodes( 8,     13, -0.3,   0.75,   150.0,  50.0 );     //Russia, Siberia 1
    globe.LinkNodes( 12,    13, -0.25,  0.75,   150.0,  50.0 );     //Central Asia, Siberia 1
    globe.LinkNodes( 9,     12, -0.2,   0.75,   150.0,  50.0 );     //Turkey, Central Asia
    globe.LinkNodes( 9,     11, -3.0,   3.0,    150.0,  50.0 );     //Turkey, Persia
    globe.LinkNodes( 11,    12, -2.0,   2.0,    150.0,  50.0 );     //Persia, Central Asia
    globe.LinkNodes( 13,    14, -0.5,   0.75,   150.0,  50.0 );     //Siberia 1, Siberia 2
    globe.LinkNodes( 11,    16, -2.0,   2.0,    150.0,  50.0 );     //Persia, North India
    globe.LinkNodes( 16,    17, -0.5,   1.0,    150.0,  50.0 );     //North India, South India
    globe.LinkNodes( 16,    15, -0.5,   1.0,    150.0,  50.0 );     //North India, Himalayas
    globe.LinkNodes( 12,    16, -0.5,   1.0,    150.0,  50.0 );     //Central Asia, North India
    globe.LinkNodes( 12,    15, -0.2,   0.75,   150.0,  50.0 );     //Central Asia, Himalayas
    globe.LinkNodes( 16,    18, -0.5,   1.0,    150.0,  50.0 );     //North India, Burma
    globe.LinkNodes( 18,    19, -1.0,   1.5,    150.0,  50.0 );     //Burma, Indochina
    globe.LinkNodes( 19,    20, -0.5,   0.75,   150.0,  50.0 );     //Indochina, Malaysia
    globe.LinkNodes( 15,    18, -1.0,   1.5,    150.0,  50.0 );     //Himalayas, Burma
    globe.LinkNodes( 18,    21, -1.0,   1.5,    150.0,  50.0 );     //Burma, Inner China
    globe.LinkNodes( 18,    22, -1.0,   1.5,    150.0,  50.0 );     //Burma, South China
    globe.LinkNodes( 21,    22, -0.5,   2.0,    150.0,  50.0 );     //Inner China, South China
    globe.LinkNodes( 21,    15, 1.0,    2.0,    150.0,  50.0 );     //Inner China, Himalayas
    globe.LinkNodes( 22,    23, -0.5,   1.0,    150.0,  50.0 );     //South China, North China
    globe.LinkNodes( 21,    23, -0.5,   2.0,    150.0,  50.0 );     //Inner China, North China
    globe.LinkNodes( 23,    24, -0.3,   0.5,    150.0,  50.0 );     //North China, Mongolia
    globe.LinkNodes( 21,    24, -0.5,   1.5,    150.0,  50.0 );     //Inner China, Mongolia
    globe.LinkNodes( 14,    24, -0.5,   1.5,    150.0,  50.0 );     //Siberia 2, Mongolia
    globe.LinkNodes( 14,    26, -0.5,   1.0,    150.0,  50.0 );     //Siberia 2, Siberia 3
    globe.LinkNodes( 24,    26, -0.5,   1.0,    150.0,  50.0 );     //Mongolia, Siberia 3
    globe.LinkNodes( 24,    25, -0.5,   1.5,    150.0,  50.0 );     //Mongolia, Korea
    globe.LinkNodes( 25,    26, -0.5,   1.0,    150.0,  50.0 );     //Korea, Siberia 3
    globe.LinkNodes( 26,    27, -0.5,   1.0,    150.0,  50.0 );     //Siberia 3, Siberia 4

    globe.LinkNodes( 12,    29, -0.2,   0.75,   150.0,  50.0 );     //Central Asia, Xinjiang
    globe.LinkNodes( 13,    29, -0.2,   0.5,    150.0,  50.0 );     //Siberia 1, Xinjiang
    globe.LinkNodes( 14,    29, -0.3,   0.75,   150.0,  50.0 );     //Siberia 2, Xinjiang
    globe.LinkNodes( 24,    29, -0.5,   1.5,    150.0,  50.0 );     //Mongolia, Xinjiang
    globe.LinkNodes( 15,    29, 0.0,    2.5,    150.0,  50.0 );     //Himalayas, Xinjiang
    globe.LinkNodes( 21,    29, -0.5,   2.0,    150.0,  50.0 );     //Inner, Xinjiang

    cck::Vec3 colors[50];
    for ( int i = 0; i < 50; ++i )
    {
        colors[i] = cck::Vec3( rand() / ( RAND_MAX / 255 ),
                               rand() / ( RAND_MAX / 255 ),
                               rand() / ( RAND_MAX / 255 ) );
    }

    BMP heightMap;
    heightMap.SetSize( xRes, yRes );

    BMP idMap;
    idMap.SetSize( xRes, yRes );

    double height;
    int id;
    double influence;
    int terrainHeight;

    for ( int x = 0; x < xRes; x++ )
    {
        double lon = ( (double) x / xRes * 360.0 ) - 180.0;

        for ( int y = 0; y < yRes; y++ )
        {
            double lat = -( ( (double) y / yRes * 180.0 ) - 90.0 );
            //cout << lat << " " << lon << endl;

            globe.SampleData( lat, lon, height, id );
            //globe.SampleInfluence( lat, lon, influence );

            if ( height > 0.0 )
            {
                terrainHeight = (int) ( height / 6.0 * 255.0 );
            }
            else
            {
                terrainHeight = 0;
            }
            //terrainHeight = (int) ( influence / 1.0 * 255.0 );

            //cout << terrainHeight << endl;

            if ( terrainHeight > 0 )
            {
                heightMap(x,y)->Red = terrainHeight;
                heightMap(x,y)->Green = terrainHeight;
                heightMap(x,y)->Blue = terrainHeight;

                if ( id == -1 )
                {
                    idMap(x,y)->Red = 255;
                    idMap(x,y)->Green = 255;
                    idMap(x,y)->Blue = 255;
                }
                else
                {
                    idMap(x,y)->Red = colors[id].x;
                    idMap(x,y)->Green = colors[id].y;
                    idMap(x,y)->Blue = colors[id].z;
                }
            }
            else
            {
                heightMap(x,y)->Red = 255;
                heightMap(x,y)->Green = 0;
                heightMap(x,y)->Blue = 255;

                idMap(x,y)->Red = 0;
                idMap(x,y)->Green = 0;
                idMap(x,y)->Blue = 0;
            }
        }
    }

    cout << "Generated map." << endl;

    heightMap.WriteToFile( "height.bmp" );
    idMap.WriteToFile( "id.bmp" );

    cout << "Wrote map to file." << endl;
    return 0;
}
