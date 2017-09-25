import nltk
import numpy as np
from nltk.corpus import wordnet
from scipy.stats import expon
from nltk.tokenize import word_tokenize


class Speech:
    def __init__(self):
        self.objects = ["cube one", "cube two", "plate", "yellow banana ", "coffee"]
        self.epsilon = len(self.objects)
        self.grammar = nltk.CFG.fromstring("""
          S -> NP VP
          VP -> V NP | V NP PP
          PP -> P NP
          V -> "pick" | "put"
          NP -> Det N | Det N PP | N
          Det -> "a" | "an" | "the" | "my" | "this"
          N -> "cube" | "plate" | "banana" | "coffee" | "Fetch" | "object" | "Bob" | "supermarket"
          P -> "behind" | "near" | "left to" | "right to"
          """)
        self.tags =  {"NOUN": ".n.01", "VERB": ".v.01", "ADP": ".r.01", "ADJ": ".n.01", "NUM": ".n.01"}
        self.coordinates = np.zeros((self.epsilon, 3))

    def remove_det(self, sent):
        taged = nltk.pos_tag(sent)
        for tag in taged:
            if tag[1] == "DT":
                sent.remove(tag[0])
        return sent

    def joint_score(self, object, word):
        sum_score = 0
        w1 = wordnet.synset(word + self.tags[nltk.pos_tag(word.split(), "universal")[0][1]])
        for w in object.split():
            w2 = wordnet.synset(w + self.tags[nltk.pos_tag(w.split(), "universal")[0][1]])
            sum_score += w1.wup_similarity(w2)

        return sum_score/len(object.split())

    def joint_score_sum(self, word):
        sum_score = 0
        for i in range(0, self.epsilon):
            wi = self.joint_score(self.objects[i], word)
            sum_score += wi
        return sum_score

    def get_coordinates(self, object):
        index = self.objects.index(object)
        print index
        print self.coordinates
        return self.coordinates[index, :]

    def near(self, target, reference):
        target_coordinates = self.get_coordinates(target)
        reference_coordinates = self.get_coordinates(reference)
        print target, np.linalg.norm(np.exp(-abs(target_coordinates - reference_coordinates)))
        return np.linalg.norm(np.exp(-abs(target_coordinates - reference_coordinates)))

    def far(self, target, reference):
        target_coordinates = self.get_coordinates(target)
        reference_coordinates = self.get_coordinates(reference)
        print target, np.linalg.norm(np.exp(abs(target_coordinates - reference_coordinates)))
        return np.linalg.norm(np.exp(abs(target_coordinates - reference_coordinates)))

    def preposition_locator(self, preposition, reference):
        function = {"near": self.near,
                    "far": self.far
                    }
        p = np.zeros((self.epsilon, 1))
        for i in range(0, self.epsilon):
            p[i] = function[preposition](self.objects[i], reference)
        p = p/sum(p)
        return p

    def get_constituents(self, sentance, type):
        rd_parser = nltk.RecursiveDescentParser(self.grammar)
        words = sentance.split()
        for tree in rd_parser.parse(words):
            p = [" ".join(i.leaves()) for i in tree.subtrees() if i.label() == type]
        return p

    def get_grounding_probability(self, grounding, object, alpha=0.005):
        grounding = self.remove_det(grounding.split())
        p = 1
        for i in range(0, len(grounding)):
            p *= (self.joint_score(object, grounding[i]) + alpha)/(self.joint_score_sum(grounding[i]) + alpha*self.epsilon)
        return p

    def get_grounding_distribution(self, grounding):
        grounding_distribution = np.zeros((self.epsilon, 1))
        for i in range(0, self.epsilon):
            grounding_distribution[i] = self.get_grounding_probability(grounding, self.objects[i])
        return grounding_distribution/sum(grounding_distribution)

    def get_preposition_distribution(self, preposition_phrase):
        taged = nltk.pos_tag(preposition_phrase.split(), "universal")
        preposition = taged[0][0]
        reference = taged[1][0]

        grounding_distribution = self.get_grounding_distribution(reference)
        argmax = int(np.argmax(grounding_distribution))

        print "Maximum probabilty for grounding is " + self.objects[argmax]

        preposition_distribution = grounding_distribution[argmax]*self.preposition_locator(preposition, self.objects[argmax])

        return preposition_distribution/sum(preposition_distribution)

    def observation_distribution(self, sentance):
        phrases = ["NP", "V", "PP"]
        d = {}
        for phrase in phrases:
            d[phrase] = self.get_constituents(sentance, phrase)
        if not d["PP"]:
            print d["NP"]
            distribution = self.get_grounding_distribution(d["NP"][1])
        else:
            for p in d["PP"]:
                distribution = self.get_preposition_distribution(p)

        return distribution


# for o in objects:
#     print nltk.pos_tag(o.split(), "universal")
# print get_grounding_distribution("the banana", objects[3])
# print nltk.pos_tag("yellow".split(), "universal")
# w1 = wordnet.synset("beach.n.01")
# w2 = wordnet.synset("sea.n.01")
# print w1.wup_similarity(w2)
# S = Speech()
#
# coordinates = np.zeros((5, 3))
# coordinates.fill(5)
# coordinates[4] = np.array([2, 2, 2])
# coordinates[3] = np.array([2, 2, 3])
#
# S.coordinates = coordinates
# print "GD", S.observation_distribution("Fetch pick this cube")
