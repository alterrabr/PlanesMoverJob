using System.Collections.Generic;
using System.Collections;
using System.Threading.Tasks;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;
using PathCreation;
using Cysharp.Threading.Tasks;

public class PerformanceTaskJob : MonoBehaviour
{
    private enum MethodType
    {
        NormalMovePlane,
        TaskMovePlane,
        MovePlaneJob,
        MovePlaneParallelJob
    }

    private enum MovePlaneJobType
    {
        ImmediateMainThread,
        ScheduleSingleWorkerThread,
        ScheduleParallelWorkerThreads
    }

    [SerializeField] private int _numberOfPlanes = 1000;
    [SerializeField] private Transform _planePrefab;
    [SerializeField] private Transform _pathPrefab;

    [SerializeField] private MethodType _method = MethodType.NormalMovePlane;
    [SerializeField] private MovePlaneJobType _movePlaneJobType = MovePlaneJobType.ImmediateMainThread;

    [SerializeField] private float _methodTime;

    private readonly List<Plane> _planes = new List<Plane>();
    private readonly List<GameObject> _paths = new List<GameObject>();

    private Vector3 _lastPoint;

    private void Start()
    {
        _numberOfPlanes = FindObjectOfType<GodScript>().GetNumberOfPlanes();

        //Time.timeScale = 4f;
        for (int i = 0; i < _numberOfPlanes; i++)
        {
            Transform enemy = Instantiate(_planePrefab,
                                          new Vector3(Random.Range(-10f, 10f), Random.Range(-8f, 8f)),
                                          Quaternion.identity);

            Transform path = Instantiate(_pathPrefab,
                              new Vector3(enemy.position.x, enemy.position.y),
                              Quaternion.identity);

            _paths.Add(path.gameObject);

            _planes.Add(new Plane { Transform = enemy, Speed = 0.5f, Bezier = GenerateRandomPath(_paths[i]), Distance = 0f, State = PlaneState.AtPointA });

            _planes[i].Transform.position = _planes[i].Bezier.GetPoint(0);

            _planes[i].Vertex = new VertexPath(_planes[i].Bezier, _paths[i].transform);
        }
    }

    private void Update()
    {
        switch (_method)
        {
            case MethodType.NormalMovePlane:
                NormalPlaneMove(Time.deltaTime);
                break;
            case MethodType.TaskMovePlane:
                break;
            case MethodType.MovePlaneJob:
            case MethodType.MovePlaneParallelJob:
                MoveEnemyJob(Time.deltaTime);
                break;

        }
    }

    private void OnDestroy()
    {
        foreach (var path in _paths)
        {
            Destroy(path.gameObject);
        }

        foreach (var plane in _planes)
        {
            Destroy(plane.Transform.gameObject);
        }

        _paths.Clear();
        _planes.Clear();
    }

    public void SetMethod(int method)
    {
        switch (method)
        {
            case 1:
                _method = MethodType.NormalMovePlane;
                break;

            case 2:
                _method = MethodType.MovePlaneJob;
                break;

            case 3:
                _method = MethodType.MovePlaneParallelJob;
                break;
        }
    }

    private void MoveEnemyJob(float deltaTime)
    {
        var containerSize = _planes.Count;

        NativeArray<float> distances = new NativeArray<float>(containerSize, Allocator.TempJob);
        NativeArray<float> lenths = new NativeArray<float>(containerSize, Allocator.TempJob);
        NativeArray<float3> positions = new NativeArray<float3>(containerSize, Allocator.TempJob);
        NativeArray<Quaternion> rotations = new NativeArray<Quaternion>(containerSize, Allocator.TempJob);
        NativeArray<float> speed = new NativeArray<float>(containerSize, Allocator.TempJob);
        NativeArray<float3> startPoints = new NativeArray<float3>(containerSize, Allocator.TempJob);
        NativeArray<float3> aControlPoints = new NativeArray<float3>(containerSize, Allocator.TempJob);
        NativeArray<float3> bControlPoints = new NativeArray<float3>(containerSize, Allocator.TempJob);
        NativeArray<float3> endPoints = new NativeArray<float3>(containerSize, Allocator.TempJob);
        NativeArray<int> states = new NativeArray<int>(containerSize, Allocator.TempJob);

        for (int i = 0; i < containerSize; i++)
        {
            distances[i] = _planes[i].Distance;
            lenths[i] = _planes[i].Vertex.length;
            positions[i] = _planes[i].Transform.position;
            rotations[i] = _planes[i].Transform.rotation;
            speed[i] = _planes[i].Speed;

            startPoints[i] = _planes[i].Bezier.GetPoint(0);
            aControlPoints[i] = _planes[i].Bezier.GetPoint(1);
            bControlPoints[i] = _planes[i].Bezier.GetPoint(2);
            endPoints[i] = _planes[i].Bezier.GetPoint(3);

            states[i] = (int)_planes[i].State;
        }

        if (_method == MethodType.MovePlaneJob)
        {
            MovePlaneJob job = new MovePlaneJob
            {
                deltaTime = deltaTime,
                Distance = distances,
                Length = lenths,
                Positions = positions,
                Rotations = rotations,
                Speed = speed,
                StartPoints = startPoints,
                AControlPoints = aControlPoints,
                BControlPoints = bControlPoints,
                EndPoints = endPoints,
                States = states
            };

            switch (_movePlaneJobType)
            {
                case MovePlaneJobType.ImmediateMainThread:
                    job.Run(_planes.Count);
                    for (int i = 0; i < containerSize; i++)
                    {
                        if (this.isActiveAndEnabled == false)
                        {
                            break;
                        }

                        _planes[i].Transform.rotation = rotations[i];
                        _planes[i].Transform.position = positions[i];
                        _planes[i].Distance = distances[i];

                        _planes[i].State = (PlaneState)states[i];

                        switch (states[i])
                        {
                            case 1:
                                DelayPlaneDeploy(_planes[i], PlaneState.MoveToB);
                                break;

                            case 3:
                                DelayPlaneDeploy(_planes[i], PlaneState.MoveToA);
                                break;
                        }
                    }

                    distances.Dispose();
                    lenths.Dispose();
                    positions.Dispose();
                    rotations.Dispose();
                    speed.Dispose();
                    startPoints.Dispose();
                    endPoints.Dispose();
                    aControlPoints.Dispose();
                    bControlPoints.Dispose();
                    states.Dispose();
                    break;

                case MovePlaneJobType.ScheduleSingleWorkerThread:
                    JobHandle scheduleJobDependency = new JobHandle();
                    JobHandle scheduleJobHandle = job.Schedule(_planes.Count, scheduleJobDependency);
                    scheduleJobHandle.Complete();
                    for (int i = 0; i < containerSize; i++)
                    {
                        if (this.isActiveAndEnabled == false)
                        {
                            break;
                        }

                        _planes[i].Transform.rotation = rotations[i];
                        _planes[i].Transform.position = positions[i];
                        _planes[i].Distance = distances[i];

                        _planes[i].State = (PlaneState)states[i];

                        switch (states[i])
                        {
                            case 1:
                                DelayPlaneDeploy(_planes[i], PlaneState.MoveToB);
                                break;

                            case 3:
                                DelayPlaneDeploy(_planes[i], PlaneState.MoveToA);
                                break;
                        }
                    }

                    distances.Dispose();
                    lenths.Dispose();
                    positions.Dispose();
                    rotations.Dispose();
                    speed.Dispose();
                    startPoints.Dispose();
                    endPoints.Dispose();
                    aControlPoints.Dispose();
                    bControlPoints.Dispose();
                    states.Dispose();
                    break;

                case MovePlaneJobType.ScheduleParallelWorkerThreads:
                    JobHandle scheduleJobDependency2 = new JobHandle();
                    JobHandle scheduleParallelJobHandle =
                        job.ScheduleParallel(_planes.Count, 1, scheduleJobDependency2);

                    scheduleParallelJobHandle.Complete();

                    for (int i = 0; i < containerSize; i++)
                    {
                        if (this.isActiveAndEnabled == false)
                        {
                            break;
                        }

                        _planes[i].Transform.rotation = rotations[i];
                        _planes[i].Transform.position = positions[i];
                        _planes[i].Distance = distances[i];

                        _planes[i].State = (PlaneState)states[i];

                        switch (states[i])
                        {
                            case 1:
                                DelayPlaneDeploy(_planes[i], PlaneState.MoveToB);
                                break;

                            case 3:
                                DelayPlaneDeploy(_planes[i], PlaneState.MoveToA);
                                break;
                        }
                    }

                    distances.Dispose();
                    lenths.Dispose();
                    positions.Dispose();
                    rotations.Dispose();
                    speed.Dispose();
                    startPoints.Dispose();
                    endPoints.Dispose();
                    aControlPoints.Dispose();
                    bControlPoints.Dispose();
                    states.Dispose();
                    break;
            }
        }
        else if (_method == MethodType.MovePlaneParallelJob)
        {
            MovePlaneParallelJob job = new MovePlaneParallelJob
            {
                deltaTime = deltaTime,
                Distance = distances,
                Length = lenths,
                Positions = positions,
                Rotations = rotations,
                Speed = speed,
                StartPoints = startPoints,
                AControlPoints = aControlPoints,
                BControlPoints = bControlPoints,
                EndPoints = endPoints,
                States = states
            };

            JobHandle jobHandle = job.Schedule(_planes.Count, _planes.Count / 10);

            jobHandle.Complete();

            for (int i = 0; i < containerSize; i++)
            {
                if (this.isActiveAndEnabled == false)
                {
                    break;
                }

                _planes[i].Transform.rotation = rotations[i];
                _planes[i].Transform.position = positions[i];
                _planes[i].Distance = distances[i];

                _planes[i].State = (PlaneState)states[i];

                switch (states[i])
                {
                    case 1:
                        DelayPlaneDeploy(_planes[i], PlaneState.MoveToB);
                        break;

                    case 3:
                        DelayPlaneDeploy(_planes[i], PlaneState.MoveToA);
                        break;
                }
            }

            distances.Dispose();
            lenths.Dispose();
            positions.Dispose();
            rotations.Dispose();
            speed.Dispose();
            startPoints.Dispose();
            endPoints.Dispose();
            aControlPoints.Dispose();
            bControlPoints.Dispose();
            states.Dispose();
        }
    }

    private BezierPath GenerateRandomPath(GameObject path)
    {
        var pathCreator = path.GetComponent<PathCreator>();

        var startPoint = new Vector3();
        var endPoint = new Vector3();

        if (_lastPoint == Vector3.zero)
        {
            startPoint = new Vector3(Random.Range(-10f, 10f), Random.Range(-5f, 5f), 0f);
            endPoint = new Vector3(Random.Range(-10f, 10f), Random.Range(-5f, 5f), 0f);
            _lastPoint = endPoint;
        }
        else
        {
            startPoint = _lastPoint;
            endPoint = new Vector3(Random.Range(-10f, 10f), Random.Range(-5f, 5f), 0f);

            while (Vector3.Distance(startPoint, endPoint) < 1f)
            {
                endPoint = new Vector3(Random.Range(-10f, 10f), Random.Range(-5f, 5f), 0f);
            }

            _lastPoint = endPoint;
        }

        float magnitude = (startPoint.magnitude + endPoint.magnitude) / 2;
        float angle = Vector3.Angle(startPoint, endPoint);
        Vector3 vect = Quaternion.Euler(0, 0, -angle / 2) * startPoint;
        vect.Normalize();
        vect *= magnitude;
        vect = ((startPoint + endPoint) / 2).normalized * (magnitude + (startPoint - endPoint).magnitude * 0.1f);

        pathCreator.bezierPath = new BezierPath(path.transform.position, false, PathSpace.xy);

        pathCreator.bezierPath.MovePoint(0, startPoint);
        pathCreator.bezierPath.MovePoint(3, endPoint);
        pathCreator.bezierPath.MovePoint(1, (vect + startPoint).normalized * (vect.magnitude + startPoint.magnitude) / 2);
        pathCreator.bezierPath.MovePoint(2, (vect + endPoint).normalized * (vect.magnitude + endPoint.magnitude) / 2);

        DrawLine(path);

        return pathCreator.bezierPath;
    }

    public void DrawLine(GameObject path)      //draw route line
    {
        LineRenderer lr = path.GetComponent<LineRenderer>();
        var points = path.GetComponent<PathCreator>().path.localPoints;

        lr.positionCount = points.Length;
        lr.startWidth = 0.05f;
        float width = lr.startWidth;
        lr.SetPositions(points);
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
    }

    private async void DelayPlaneDeploy(Plane plane, PlaneState state)
    {
        //plane.planeBehavior.SpriteRenderer.enabled = false;
        //plane.planeBehavior.collider.enabled = false;

        //switch (state)
        //{
        //    case PlaneState.MoveToB:
        //        plane.planeBehavior.UnloadingPassengers(plane.planeMover.curCityA);
        //        break;

        //    case PlaneState.MoveToA:
        //        plane.planeBehavior.UnloadingPassengers(plane.planeMover.curCityB);
        //        break;
        //}

        await UniTask.Delay(3000, ignoreTimeScale: false, cancellationToken: this.GetCancellationTokenOnDestroy()).SuppressCancellationThrow();

        plane.State = state;

        //switch (state)
        //{
        //    case PlaneState.MoveToB:
        //        plane.planeBehavior.LoadingPassengers(plane.planeMover.curCityA, plane.planeMover.curCityB);
        //        break;

        //    case PlaneState.MoveToA:
        //        plane.planeBehavior.LoadingPassengers(plane.planeMover.curCityB, plane.planeMover.curCityA);
        //        break;
        //}

        //if (plane.planeBehavior != null)
        //{
        //    plane.planeBehavior.SpriteRenderer.enabled = true;
        //    plane.planeBehavior.collider.enabled = true;

        //    plane.planeBehavior.Plane.State = state;

        //    plane.planeBehavior.Plane.StateChangeAwait = false;
        //}
    }

    private void NormalPlaneMove(float deltaTime)
    {
        for (int index = 0; index < _planes.Count; index++)
        {
            var currentDistance = 0f;
            var startPoint = _planes[index].Bezier.GetPoint(0);
            var endPoint = _planes[index].Bezier.GetPoint(3);
            var AControlPoint = _planes[index].Bezier.GetPoint(1);
            var BControlPoint = _planes[index].Bezier.GetPoint(2);
            var length = _planes[index].Vertex.length;
            var position = _planes[index].Transform.position;
            var rotation = _planes[index].Transform.rotation;
            var distance = _planes[index].Distance;
            var speed = _planes[index].Speed;

            switch (_planes[index].State)
            {
                case PlaneState.AtPointA:
                    _planes[index].Transform.position = startPoint;
                    DelayPlaneDeploy(_planes[index], PlaneState.MoveToB);
                    continue;

                case PlaneState.MoveToB:
                    currentDistance = distance + speed * deltaTime;
                    break;

                case PlaneState.AtPointB:
                    _planes[index].Transform.position = endPoint;
                    DelayPlaneDeploy(_planes[index], PlaneState.MoveToA);
                    continue;

                case PlaneState.MoveToA:
                    currentDistance = distance - speed * deltaTime;
                    break;
            }

            var t = Mathf.InverseLerp(0, length, currentDistance);

            float u = 1f - t;
            float t2 = t * t;
            float u2 = u * u;
            float u3 = u2 * u;
            float t3 = t2 * t;

            Vector3 result =
                (u3) * startPoint +
                (3f * u2 * t) * AControlPoint +
                (3f * u * t2) * BControlPoint +
                (t3) * endPoint;

            Vector3 xDirection = Vector3.Normalize(result - position);

            Vector3 yDirection = Quaternion.Euler(0, 0, 90) * xDirection;

            Vector3 zDirection = Vector3.forward;

            switch (_planes[index].State)
            {
                // Plane at city A
                case PlaneState.AtPointA:
                    continue;

                // Plane moving to city B
                case PlaneState.MoveToB:
                    if (Vector3.Distance(position, endPoint) < 0.02f || distance > length)
                    {
                        _planes[index].State = PlaneState.AtPointB;
                    }
                    break;

                // Plane at city B
                case PlaneState.AtPointB:
                    continue;

                // Plane moving to city B
                case PlaneState.MoveToA:
                    if (Vector3.Distance(position, startPoint) < 0.02f || distance < 0)
                    {
                        _planes[index].State = PlaneState.AtPointA;
                    }
                    break;
            }

            _planes[index].Distance = currentDistance;

            _planes[index].Transform.rotation = Quaternion.LookRotation(zDirection, yDirection) * Quaternion.Euler(0, 0, -90);

            _planes[index].Transform.position = result;

            PerformanceIntensiveMethod(1000);
        }
    }

    private static void PerformanceIntensiveMethod(int timesToRepeat)
    {

        float value = 0f;

        for (int i = 0; i < timesToRepeat; i++)
        {
            value = Mathf.Exp(Mathf.Sqrt(value));
        }
    }
}
