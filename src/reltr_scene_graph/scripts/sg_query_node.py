#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, re, json, threading
import rospy
from std_msgs.msg import String, Int32
from collections import Counter

# ----- LLM 옵셔널 의존성 (없으면 규칙모드로 동작)
OPENAI_OK = False
try:
    import openai  # pip install openai
    OPENAI_OK = True
except Exception:
    pass

def simple_lemmatize(noun: str) -> str:
    n = noun.strip().lower()
    # 아주 얕은 표제화 (규칙 기반)
    if n.endswith("ies"):  # e.g., 'bodies' -> 'body'
        return n[:-3] + "y"
    if n.endswith("es") and not n.endswith("ses"):  # 'boxes'->'box'
        return n[:-2]
    if n.endswith("s") and n not in ("glass", "class"):
        return n[:-1]
    return n

def parse_how_many_rule(q: str):
    """
    간단 규칙: 'how many <NOUN> ... ?' -> category
    """
    m = re.search(r"how\s+many\s+([a-zA-Z_]+)", q.strip().lower())
    if not m:
        return None
    cat = simple_lemmatize(m.group(1))
    return {"task": "count_category", "category": cat}

def parse_with_llm(q: str, model="gpt-4o-mini"):
    """
    LLM에게 'JSON으로만' 의도 뽑으라고 시킴.
    schema:
      {"task":"count_category", "category":"chair"}
    """
    if not OPENAI_OK or not os.getenv("OPENAI_API_KEY"):
        return None

    openai.api_key = os.getenv("OPENAI_API_KEY")

    system_msg = (
        "You convert an English question about a scene into a STRICT JSON intent.\n"
        "Only reply with JSON, no prose.\n"
        "Schema: {\"task\":\"count_category\",\"category\":\"<noun>\"}\n"
        "Examples:\n"
        "Q: How many chairs? -> {\"task\":\"count_category\",\"category\":\"chair\"}\n"
        "Q: How many tables are there? -> {\"task\":\"count_category\",\"category\":\"table\"}\n"
        "If question is not a counting question, reply: {\"task\":\"unknown\"}"
    )
    user_msg = q

    try:
        resp = openai.chat.completions.create(
            model=model,
            messages=[
                {"role":"system","content":system_msg},
                {"role":"user","content":user_msg}
            ],
            temperature=0.0,
        )
        txt = resp.choices[0].message.content.strip()
        # JSON만 오도록 시켰지만 혹시 몰라 보호
        start = txt.find("{"); end = txt.rfind("}")
        if start != -1 and end != -1:
            txt = txt[start:end+1]
        intent = json.loads(txt)
        # 가벼운 정리
        if intent.get("task") == "count_category" and "category" in intent:
            intent["category"] = simple_lemmatize(intent["category"])
        return intent
    except Exception as e:
        rospy.logwarn(f"[sg_query_node] LLM parse failed, fallback to rules: {e}")
        return None

class SGQueryNode:
    def __init__(self):
        rospy.init_node("sg_query_node")

        # ----- params
        self.path_json = rospy.get_param("~path",
            os.path.expanduser("~/.ros/scene_graph_results/final_sg.json"))
        self.sg_topic = rospy.get_param("~sg_topic", "/final_scene_graph/json")  # latched JSON 받기
        self.query_topic = rospy.get_param("~query_topic", "/challenge_question")  # 챌린지 규약
        self.answer_text_topic = rospy.get_param("~answer_text_topic", "/scene_graph/answer")
        self.use_llm = rospy.get_param("~use_llm", False)
        self.llm_model = rospy.get_param("~llm_model", "gpt-4o-mini")

        # ----- publishers
        self.pub_num = rospy.Publisher("/numerical_response", Int32, queue_size=10)  # 챌린지 규약
        self.pub_ans = rospy.Publisher(self.answer_text_topic, String, queue_size=10)

        # ----- scene graph 로딩
        self.nodes = []
        self.edges = []
        self.cat_counts = Counter()
        self.lock = threading.Lock()

        # (A) 토픽에서 latched로 받기(권장)
        rospy.Subscriber(self.sg_topic, String, self._on_sg, queue_size=1)
        # (B) 파일이 있으면 선로딩(토픽이 나중에 와도 토픽으로 갱신됨)
        if os.path.exists(self.path_json):
            try:
                with open(self.path_json, "r") as f:
                    sg = json.load(f)
                self._set_sg(sg)
                rospy.loginfo("[sg_query_node] loaded SG from file: %s", self.path_json)
            except Exception as e:
                rospy.logwarn(f"[sg_query_node] failed to load file SG: {e}")

        # ----- query subscriber
        rospy.Subscriber(self.query_topic, String, self._on_query, queue_size=10)
        rospy.loginfo("[sg_query_node] ready. use_llm=%s model=%s",
                      self.use_llm, self.llm_model)

    def _set_sg(self, sg):
        with self.lock:
            self.nodes = sg.get("nodes", [])
            self.edges = sg.get("edges", [])
            self.cat_counts = Counter([n.get("orig_id","unknown").lower() for n in self.nodes])

    def _on_sg(self, msg: String):
        try:
            sg = json.loads(msg.data)
            self._set_sg(sg)
            rospy.loginfo("[sg_query_node] SG updated from topic (%d nodes, %d edges)",
                          len(self.nodes), len(self.edges))
        except Exception as e:
            rospy.logwarn(f"[sg_query_node] bad SG on topic: {e}")

    def _on_query(self, msg: String):
        q = msg.data.strip()
        intent = None

        if self.use_llm:
            intent = parse_with_llm(q, model=self.llm_model)

        if intent is None:
            intent = parse_how_many_rule(q)  # 영어 전용 간단 규칙

        if not intent or intent.get("task") != "count_category":
            txt = "Unsupported question. Try: 'How many chairs?'"
            self.pub_ans.publish(String(txt))
            rospy.logwarn("[sg_query_node] unknown query: %s", q)
            return

        cat = intent["category"]
        with self.lock:
            count = int(self.cat_counts.get(cat, 0))
        # 챌린지 규약: 숫자는 여기로!
        self.pub_num.publish(Int32(data=count))
        # 텍스트 응답은 참고용
        self.pub_ans.publish(String(f"{cat}: {count}"))

def main():
    SGQueryNode()
    rospy.spin()

if __name__ == "__main__":
    main()
