package org.firstinspires.ftc.teamcode.input;

import java.util.ArrayList;

public class Async {
    private Body body;
    private ArrayList<Dependency> dependencies;
    private ArrayList<Async> dependents;

    public Async(Body body) {
        this.body = body;
        this.dependencies = new ArrayList<>();
        this.dependents = new ArrayList<>();
    }

    public static interface Body {
        public void run(Async async);
    }

    public static class Dependency {
        public Async async;
        public boolean fulfilled;

        public Dependency(Async async) {
            this.async = async;
            this.fulfilled = false;
        }
    }

    private void addDependency(Async async) {
        this.dependencies.add(new Dependency(async));
    }

    private void addDependent(Async async) {
        this.dependents.add(async);
    }

    public Async then(Async that) {
        this.addDependent(that);
        that.addDependency(this);
        return that;
    }

    public Async then(Body that) {
        Async thatAsync = new Async(that);
        this.addDependent(thatAsync);
        thatAsync.addDependency(this);
        return thatAsync;
    }

//    public Async sleep(double ms) {
//    }

    public void run() {
        this.body.run(this);
    }

    public void fulfill() {
        for (Async dependent : this.dependents)
            dependent.update(this);
    }

    private void update(Async async) {
        boolean fulfilled = true;
        for (Dependency dependency : this.dependencies) {
            if (async == dependency.async) dependency.fulfilled = true;
            fulfilled = fulfilled && dependency.fulfilled;
        }
        if (fulfilled) this.run();
    }
}
