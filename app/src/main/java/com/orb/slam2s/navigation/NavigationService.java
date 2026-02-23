package com.orb.slam2s.navigation;

import android.content.Context;
import android.opengl.Matrix;
import android.util.Log;

import com.orb.slam2s.slamar.NativeHelper;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

public class NavigationService {
    private static final String TAG = "NavigationService";

    public static class Node {
        public String id;
        public String name;
        public String type;
        public float x;
        public float y;
        public float z;
    }

    public static class Edge {
        public String from;
        public String to;
        public float cost;
        public boolean bidirectional;
    }

    public static class Poi {
        public String id;
        public String name;
        public String nodeId;
        public String category;
    }

    private final NativeHelper nativeHelper;
    private final File mapDirectory;

    private final Map<String, Node> nodes = new HashMap<>();
    private final List<Edge> edges = new ArrayList<>();
    private final Map<String, Poi> pois = new HashMap<>();

    public NavigationService(Context context, NativeHelper nativeHelper) {
        this.nativeHelper = nativeHelper;
        this.mapDirectory = new File(context.getExternalFilesDir(null), "SLAM/maps");
        if (!mapDirectory.exists()) {
            mapDirectory.mkdirs();
        }
    }

    public synchronized void clear() {
        nodes.clear();
        edges.clear();
        pois.clear();
    }

    public synchronized Node addNodeAtCurrentPose(String id, String name, String type) {
        float[] p = getCurrentCameraWorldPosition();
        Node n = new Node();
        n.id = normalizeId(id, "node_" + (nodes.size() + 1));
        n.name = name == null ? n.id : name;
        n.type = type == null ? "waypoint" : type;
        n.x = p[0];
        n.y = p[1];
        n.z = p[2];
        nodes.put(n.id, n);
        return n;
    }

    public synchronized Poi addPoi(String id, String name, String nodeId, String category) {
        if (!nodes.containsKey(nodeId)) {
            throw new IllegalArgumentException("node not found: " + nodeId);
        }
        Poi poi = new Poi();
        poi.id = normalizeId(id, "poi_" + (pois.size() + 1));
        poi.name = name == null ? poi.id : name;
        poi.nodeId = nodeId;
        poi.category = category == null ? "default" : category;
        pois.put(poi.id, poi);
        return poi;
    }

    public synchronized Edge addEdge(String from, String to, Float cost, boolean bidirectional) {
        if (!nodes.containsKey(from) || !nodes.containsKey(to)) {
            throw new IllegalArgumentException("from/to node not found");
        }
        Edge e = new Edge();
        e.from = from;
        e.to = to;
        e.bidirectional = bidirectional;
        if (cost != null && cost > 0f) {
            e.cost = cost;
        } else {
            Node a = nodes.get(from);
            Node b = nodes.get(to);
            e.cost = distance(a, b);
        }
        edges.add(e);
        return e;
    }

    public synchronized JSONObject computeRouteToPoi(String poiId) throws JSONException {
        Poi poi = pois.get(poiId);
        if (poi == null) {
            throw new IllegalArgumentException("poi not found: " + poiId);
        }

        String startNodeId = findNearestNodeId(getCurrentCameraWorldPosition());
        if (startNodeId == null) {
            throw new IllegalStateException("no start node available");
        }

        List<String> path = dijkstra(startNodeId, poi.nodeId);
        JSONArray pathArr = new JSONArray();
        JSONArray coordsArr = new JSONArray();
        float total = 0f;

        for (int i = 0; i < path.size(); i++) {
            String nid = path.get(i);
            Node n = nodes.get(nid);
            pathArr.put(nid);
            JSONObject c = new JSONObject();
            c.put("id", n.id);
            c.put("name", n.name);
            c.put("x", n.x);
            c.put("y", n.y);
            c.put("z", n.z);
            coordsArr.put(c);
            if (i > 0) {
                Node prev = nodes.get(path.get(i - 1));
                total += distance(prev, n);
            }
        }

        JSONObject obj = new JSONObject();
        obj.put("startNode", startNodeId);
        obj.put("targetPoi", poiId);
        obj.put("targetNode", poi.nodeId);
        obj.put("distanceMeters", total);
        obj.put("path", pathArr);
        obj.put("pathNodes", coordsArr);
        return obj;
    }

    public synchronized JSONObject toJson() throws JSONException {
        JSONObject root = new JSONObject();
        root.put("version", 1);

        JSONArray nodeArr = new JSONArray();
        List<String> ids = new ArrayList<>(nodes.keySet());
        Collections.sort(ids);
        for (String id : ids) {
            Node n = nodes.get(id);
            JSONObject o = new JSONObject();
            o.put("id", n.id);
            o.put("name", n.name);
            o.put("type", n.type);
            o.put("x", n.x);
            o.put("y", n.y);
            o.put("z", n.z);
            nodeArr.put(o);
        }
        root.put("nodes", nodeArr);

        JSONArray edgeArr = new JSONArray();
        for (Edge e : edges) {
            JSONObject o = new JSONObject();
            o.put("from", e.from);
            o.put("to", e.to);
            o.put("cost", e.cost);
            o.put("bidirectional", e.bidirectional);
            edgeArr.put(o);
        }
        root.put("edges", edgeArr);

        JSONArray poiArr = new JSONArray();
        List<String> poiIds = new ArrayList<>(pois.keySet());
        Collections.sort(poiIds);
        for (String id : poiIds) {
            Poi p = pois.get(id);
            JSONObject o = new JSONObject();
            o.put("id", p.id);
            o.put("name", p.name);
            o.put("nodeId", p.nodeId);
            o.put("category", p.category);
            poiArr.put(o);
        }
        root.put("pois", poiArr);
        return root;
    }

    public synchronized void fromJson(JSONObject root) throws JSONException {
        clear();
        JSONArray nodeArr = root.optJSONArray("nodes");
        if (nodeArr != null) {
            for (int i = 0; i < nodeArr.length(); i++) {
                JSONObject o = nodeArr.getJSONObject(i);
                Node n = new Node();
                n.id = o.getString("id");
                n.name = o.optString("name", n.id);
                n.type = o.optString("type", "waypoint");
                n.x = (float) o.getDouble("x");
                n.y = (float) o.getDouble("y");
                n.z = (float) o.getDouble("z");
                nodes.put(n.id, n);
            }
        }

        JSONArray edgeArr = root.optJSONArray("edges");
        if (edgeArr != null) {
            for (int i = 0; i < edgeArr.length(); i++) {
                JSONObject o = edgeArr.getJSONObject(i);
                Edge e = new Edge();
                e.from = o.getString("from");
                e.to = o.getString("to");
                e.cost = (float) o.getDouble("cost");
                e.bidirectional = o.optBoolean("bidirectional", true);
                edges.add(e);
            }
        }

        JSONArray poiArr = root.optJSONArray("pois");
        if (poiArr != null) {
            for (int i = 0; i < poiArr.length(); i++) {
                JSONObject o = poiArr.getJSONObject(i);
                Poi p = new Poi();
                p.id = o.getString("id");
                p.name = o.optString("name", p.id);
                p.nodeId = o.getString("nodeId");
                p.category = o.optString("category", "default");
                pois.put(p.id, p);
            }
        }
    }

    public synchronized File saveToMap(String mapName) throws Exception {
        String sanitized = mapName.replaceAll("[^a-zA-Z0-9_\\-]", "_");
        File f = new File(mapDirectory, sanitized + ".nav.json");
        byte[] data = toJson().toString(2).getBytes(StandardCharsets.UTF_8);
        try (FileOutputStream fos = new FileOutputStream(f)) {
            fos.write(data);
        }
        return f;
    }

    public synchronized File loadFromMap(String mapName) throws Exception {
        String sanitized = mapName.replaceAll("[^a-zA-Z0-9_\\-]", "_");
        File f = new File(mapDirectory, sanitized + ".nav.json");
        if (!f.exists()) {
            throw new IllegalArgumentException("nav file not found: " + f.getName());
        }
        byte[] buf = new byte[(int) f.length()];
        try (FileInputStream fis = new FileInputStream(f)) {
            int read = fis.read(buf);
            if (read <= 0) {
                throw new IllegalStateException("empty nav file");
            }
        }
        fromJson(new JSONObject(new String(buf, StandardCharsets.UTF_8)));
        return f;
    }

    private String normalizeId(String id, String fallback) {
        String v = (id == null || id.trim().isEmpty()) ? fallback : id.trim();
        return v.replaceAll("[^a-zA-Z0-9_\\-]", "_");
    }

    private float[] getCurrentCameraWorldPosition() {
        float[] view = new float[16];
        nativeHelper.getV(view);
        float[] inv = new float[16];
        boolean ok = Matrix.invertM(inv, 0, view, 0);
        if (!ok) {
            Log.w(TAG, "invert view matrix failed, fallback origin");
            return new float[]{0f, 0f, 0f};
        }
        return new float[]{inv[12], inv[13], inv[14]};
    }

    private String findNearestNodeId(float[] pos) {
        String best = null;
        float bestDist = Float.MAX_VALUE;
        for (Node n : nodes.values()) {
            float dx = n.x - pos[0];
            float dy = n.y - pos[1];
            float dz = n.z - pos[2];
            float d = dx * dx + dy * dy + dz * dz;
            if (d < bestDist) {
                bestDist = d;
                best = n.id;
            }
        }
        return best;
    }

    private List<String> dijkstra(String start, String goal) {
        Map<String, List<Edge>> graph = new HashMap<>();
        for (Edge e : edges) {
            graph.computeIfAbsent(e.from, k -> new ArrayList<>()).add(e);
            if (e.bidirectional) {
                Edge back = new Edge();
                back.from = e.to;
                back.to = e.from;
                back.cost = e.cost;
                back.bidirectional = true;
                graph.computeIfAbsent(back.from, k -> new ArrayList<>()).add(back);
            }
        }

        Map<String, Float> dist = new HashMap<>();
        Map<String, String> prev = new HashMap<>();
        Set<String> visited = new HashSet<>();

        PriorityQueue<String> pq = new PriorityQueue<>((a, b) ->
                Float.compare(dist.getOrDefault(a, Float.MAX_VALUE), dist.getOrDefault(b, Float.MAX_VALUE)));

        dist.put(start, 0f);
        pq.add(start);

        while (!pq.isEmpty()) {
            String u = pq.poll();
            if (!visited.add(u)) {
                continue;
            }
            if (u.equals(goal)) {
                break;
            }
            for (Edge e : graph.getOrDefault(u, Collections.emptyList())) {
                float nd = dist.get(u) + e.cost;
                if (nd < dist.getOrDefault(e.to, Float.MAX_VALUE)) {
                    dist.put(e.to, nd);
                    prev.put(e.to, u);
                    pq.add(e.to);
                }
            }
        }

        if (!start.equals(goal) && !prev.containsKey(goal)) {
            throw new IllegalStateException(String.format(Locale.US, "route not found: %s -> %s", start, goal));
        }

        List<String> path = new ArrayList<>();
        String cur = goal;
        path.add(cur);
        while (!cur.equals(start)) {
            cur = prev.get(cur);
            if (cur == null) {
                throw new IllegalStateException("path reconstruction failed");
            }
            path.add(cur);
        }
        Collections.reverse(path);
        return path;
    }

    private float distance(Node a, Node b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
}
