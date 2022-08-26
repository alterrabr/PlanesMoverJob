using UnityEngine;
using PathCreation;

public class Plane
{
    public Transform Transform;
    public float Speed;
    public BezierPath Bezier;
    public VertexPath Vertex;
    public float Distance;
    public PlaneState State;
}

public enum PlaneState
{
    Idle = 0,
    AtPointA = 1,
    MoveToB = 2,
    AtPointB = 3,
    MoveToA = 4
}