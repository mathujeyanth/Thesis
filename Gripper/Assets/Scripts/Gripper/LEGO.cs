using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LEGO : ObjectBasic
{
    public bool debug = false;
    public List<string> connectedList = new List<string>();
    HashSet<string> connectedSet = new HashSet<string>();
    public override float height {get;} = 0.096f;      
    void OnTriggerStay(Collider other) {

        if(other.CompareTag("brickNob")){
            //Debug.Log("inc"+other.transform.parent.name+":"+other.transform.name);
            if(debug && connectedSet.Add(other.transform.parent.name+"_"+other.transform.name)){
                connectedList.Add(other.transform.parent.name+"_"+other.transform.name);
            }
            IsConnected=true;
        }
    }
    void OnTriggerExit(Collider other) {

        if(other.CompareTag("brickNob")){
            if(debug && connectedSet.Remove(other.transform.parent.name+"_"+other.transform.name)){
                connectedList.Remove(other.transform.parent.name+"_"+other.transform.name);
            }
            IsConnected=false;
        }
    }
}
