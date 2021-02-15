/* A script that make any attached object smoothly patrol through designated world points in the XZ plane, using
   centripetal Catmull-Rom splines. Any number of patrol points can be created and edited in the inspector. */

using System.Collections.Generic;
using UnityEngine;

public class PatrolPath : MonoBehaviour
{
    public Vector2[] patrolPoints;  // points through which the robot will pass
    public Vector2 finalPosition;   // final position at the end of patrol

	Vector2 startGuide;     // reference Catmull point for initial position
	Vector2 endGuide;       // reference Catmull points for final position

	Vector2 initialPosV2;   // initial position in 2D plane
	Vector2 offset;         // an arbitrary offset to create Catmull reference points

	public int pathResolution = 120;    // number of points between two patrol points

    bool entropy = true;    // Whether the object is moving forwards or backwards

	List<Vector2[]> listOfCatmullNodeArrays = new List<Vector2[]>();
    // list of V2 arrays for each Catmull-Rom spline

	List<Vector3[]> pathList = new List<Vector3[]>();
    // list of V3 path arrays for each patrol point pair

    float initPosX, initPosY, initPosZ;
    // components of initial position of the robot

	Vector2[] catmullCheckpoints;
    // all Catmull points including startGuide, initialPosV2, patrolPoints[], finalPosition, endGuide 

	Vector3[] catmullPathPoints;
	// discrete path points between two patrol points

	Vector3[] allTheWay;
    // discrete points of the whole path, obtained by merging pathList arrays

	int count = 0;  // index of each discrete points in allTheWay array

	int totalNumberOfPoints;

	private void Start()
    {
		initPosX = transform.position.x;
		initPosY = transform.position.y;
		initPosZ = transform.position.z;

		initialPosV2 = new Vector2(initPosX, initPosZ);
		offset = new Vector2(1f, 0f);

		catmullCheckpoints = new Vector2[patrolPoints.Length + 4];

		startGuide = initialPosV2 - offset;
		endGuide = finalPosition + offset;

        int last = catmullCheckpoints.Length - 1;

		catmullCheckpoints[0] = startGuide;
		catmullCheckpoints[1] = initialPosV2;
		catmullCheckpoints[last - 1] = finalPosition;
		catmullCheckpoints[last] = endGuide;

        for (int i = 0; i < patrolPoints.Length; i++)
        {
			catmullCheckpoints[i + 2] = initialPosV2 + patrolPoints[i];
        }

		for (int i = 0; i < catmullCheckpoints.Length - 3; i++)
        {
			Vector2[] catmullNodes = new Vector2[4];

			for (int j = 0; j < 4; j++)
				catmullNodes[j] = catmullCheckpoints[i + j];

			listOfCatmullNodeArrays.Add(catmullNodes);
        }

        for (int i = 0; i < listOfCatmullNodeArrays.Count; i++)
        {
			var temp = CatmullRom(listOfCatmullNodeArrays[i]);
			catmullPathPoints = new Vector3[pathResolution];

            for (int j = 0; j < pathResolution; j++)
            {
				float tempx = temp[j].x;
				float tempz = temp[j].y;
				catmullPathPoints[j] = new Vector3(tempx, initPosY, tempz);
            }
			pathList.Add(catmullPathPoints);
		}

		totalNumberOfPoints = pathList.Count * pathResolution;
		allTheWay = new Vector3[totalNumberOfPoints];

        for (int i = 0; i < pathList.Count; i++)
        {
            for (int j = 0; j < pathResolution; j++)
            {
				allTheWay[i * pathResolution + j] = pathList[i][j];
            }
        }
	}

    private void Update()
    {
		if (entropy)
		{
			transform.position = allTheWay[count];
			count++;
			if (count == totalNumberOfPoints - 1)
				entropy = false;
		}
        else
        {
			transform.position = allTheWay[count];
			count--;
			if (count == 0)
				entropy = true;
        }
	}

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * CatmullRom() and GetT() below are taken from https://en.wikipedia.org/wiki/Centripetal_Catmull–Rom_spline *
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	List<Vector2> CatmullRom(Vector2[] catmullPoints)
	{
		Vector2 p0 = catmullPoints[0];
		Vector2 p1 = catmullPoints[1];
		Vector2 p2 = catmullPoints[2];
		Vector2 p3 = catmullPoints[3];

		List<Vector2> discretePoints = new List<Vector2>();

		float t0 = 0f;
		float t1 = GetT(t0, p0, p1);
		float t2 = GetT(t1, p1, p2);
		float t3 = GetT(t2, p2, p3);

		for (float t = t1; t < t2; t += ((t2 - t1) / (float)pathResolution))
		{
			Vector2 A1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
			Vector2 A2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
			Vector2 A3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;

			Vector2 B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2;
			Vector2 B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3;

			Vector2 C = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2;

			discretePoints.Add(C);
		}
		return discretePoints;
	}

	float GetT(float t, Vector3 p0, Vector3 p1)
	{
        // For 3D space, after converting all V2 into V3, <float a> will be:
        // float a = Mathf.Pow((p1.x - p0.x), 2f) + Mathf.Pow((p1.y - p0.y), 2f) + Mathf.Pow((p1.z - p0.z), 2f);

		float a = Mathf.Pow((p1.x - p0.x), 2f) + Mathf.Pow((p1.y - p0.y), 2f);
		float b = Mathf.Pow(a, 0.5f);
		float c = Mathf.Pow(b, 0.5f);

		return (c + t);
	}
}
