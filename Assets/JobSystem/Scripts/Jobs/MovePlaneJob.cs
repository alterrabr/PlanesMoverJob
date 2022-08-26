using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
public struct MovePlaneJob : IJobFor
{
    public float deltaTime;

    [NativeDisableParallelForRestriction]
    public NativeArray<float3> Positions;

    [NativeDisableParallelForRestriction]
    public NativeArray<Quaternion> Rotations;

    [NativeDisableParallelForRestriction]
    public NativeArray<int> States;

    [NativeDisableParallelForRestriction]
    public NativeArray<float> Distance;

    [ReadOnly]
    public NativeArray<float> Speed;

    [ReadOnly]
    public NativeArray<float> Length;

    [ReadOnly]
    public NativeArray<float3> StartPoints;

    [ReadOnly]
    public NativeArray<float3> AControlPoints;

    [ReadOnly]
    public NativeArray<float3> BControlPoints;

    [ReadOnly]
    public NativeArray<float3> EndPoints;

    public void Execute(int index)
    {
        var currentDistance = 0f;

        switch (States[index])
        {
            case 1:
                Positions[index] = StartPoints[index];
                return;

            case 2:
                currentDistance = Distance[index] + Speed[index] * deltaTime;
                break;

            case 3:
                Positions[index] = EndPoints[index];
                return;

            case 4:
                currentDistance = Distance[index] - Speed[index] * deltaTime;
                break;
        }

        var t = math.unlerp(0, Length[index], currentDistance);

        Distance[index] = currentDistance;

        float u = 1f - t;
        float t2 = t * t;
        float u2 = u * u;
        float u3 = u2 * u;
        float t3 = t2 * t;

        float3 result =
            (u3) * StartPoints[index] +
            (3f * u2 * t) * AControlPoints[index] +
            (3f * u * t2) * BControlPoints[index] +
            (t3) * EndPoints[index];

        // the direction we want the X axis to face (from this object, towards the target)
        float3 xDirection = math.normalize(result - Positions[index]);

        // Y axis is 90 degrees away from the X axis
        Vector3 yDirection = Quaternion.Euler(0, 0, 90) * xDirection;

        // Z should stay facing forward for 2D objects
        Vector3 zDirection = Vector3.forward;

        // apply the rotation to this object
        Rotations[index] = Quaternion.LookRotation(zDirection, yDirection) * Quaternion.Euler(0, 0, -90);

        Positions[index] = result;

        switch (States[index])
        {
            // Plane at city A
            case 1:
                return;

            // Plane moving to city B
            case 2:
                if (Vector3.Distance(Positions[index], EndPoints[index]) < 0.02f || Distance[index] > Length[index])
                {
                    States[index] = 3;
                }
                break;

            // Plane at city B
            case 3:
                return;

            // Plane moving to city B
            case 4:
                if (Vector3.Distance(Positions[index], StartPoints[index]) < 0.02f || Distance[index] < 0)
                {
                    States[index] = 1;
                }
                break;
        }

        float value = 0f;
        for (int i = 0; i < 1000; i++)
        {
            value = math.exp10(math.sqrt(value));
        }
    }
}