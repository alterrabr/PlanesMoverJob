using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using System;

public class GodScript : MonoBehaviour
{
    [SerializeField] private GameObject _gameObject;
    [SerializeField] private TMP_InputField _textMeshProUGUI;

    private PerformanceTaskJob _job;

    private GameObject _GORef;

    private void Start()
    {
        _GORef = Instantiate(_gameObject);

        _job = _GORef.GetComponent<PerformanceTaskJob>();
    }

    public void Clear()
    {
        Destroy(_GORef);
        Start();
    }

    public int GetNumberOfPlanes()
    {
        return Int32.Parse(_textMeshProUGUI.text);
    }

    public void ChangeMode(int mode)
    {
        _job.SetMethod(mode);
    }
}
