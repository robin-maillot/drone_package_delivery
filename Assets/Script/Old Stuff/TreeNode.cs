using System.Collections.Generic;
using UnityEngine;

class TreeNode : Node
{
    public Vector2 vel;
    public List<TreeNode> children;
    public TreeNode parent;
    public bool isDubins;
    public TreeNode()
    {
        this.pos = new Vector2();
        this.vel = new Vector2();
        this.time = 0F;
        this.children = new List<TreeNode>();
        this.parent = null;
        this.isDubins = false;
    }
    public TreeNode(Vector2 pos, Vector2 vel, float time)
    {
        this.pos = pos;
        this.vel = vel;
        this.time = time;
        this.children = new List<TreeNode>();
        this.parent = null;
        this.isDubins = false;
    }
    public void SetParent(TreeNode p)
    {
        if (this.parent != null) this.parent.children.Remove(this);
        this.parent = p;
    }
}