---
description: RxJS-Bedingungs-Operatoren führen Bedingungsprüfungen für Werte innerhalb von Streams durch und ermöglichen das Setzen von Standardwerten und Bedingungsbewertungen. defaultIfEmpty, every, isEmpty und mehr - zur Implementierung praktischer Szenarien wie Verarbeitung leerer Streams, vollständige Prüfungen und Existenzprüfungen zusammen mit der Typsicherheit von TypeScript.
---

# Bedingungs-Operatoren

RxJS-Bedingungs-Operatoren dienen der **Bedingungsprüfung und -bewertung** von Werten in Streams.
Sie können in praktischen Szenarien wie dem Setzen von Standardwerten für leere Streams oder der Überprüfung, ob alle Werte eine Bedingung erfüllen, eingesetzt werden.

Diese Seite stellt jeden Operator in drei Stufen vor: "Grundlegende Syntax und Verhalten", "Typische Anwendungsfälle" und "Praktische Codebeispiele (mit UI)".

Durch das Verständnis, für welche Anwendungsfälle jeder Operator geeignet ist,
und deren Kombination wird es möglich, ein robusteres und absichtskonformes reaktives Verarbeitungsdesign zu erstellen.

> [!NOTE]
> `iif` und `defer` sind **Creation Functions** (Observable-Erstellungsfunktionen), keine Bedingungs-Operatoren. Siehe [Kapitel 3: Creation Functions](/de/guide/creation-functions/) für diese.

## Operatorenliste

Die folgende Tabelle fasst die Hauptbedingungs-Operatoren und ihre Eigenschaften zusammen.

| Operator | Beschreibung |
|--------------|------|
| [defaultIfEmpty](./defaultIfEmpty.md) | Alternativer Wert, wenn keine Werte ausgegeben werden |
| [every](./every.md) | Bewertet, ob alle Werte eine Bedingung erfüllen |
| [isEmpty](./isEmpty.md) | Prüft, ob ausgegebene Werte vorhanden sind |

> **Praktische Kombinationen von Operatoren** und **anwendungsfallbasierte Anwendungsbeispiele** werden im Abschnitt [Praktische Anwendungsfälle](./practical-use-cases.md) vorgestellt.


## Achten Sie auch auf Verknüpfungen mit anderen Kategorien

Bedingungs-Operatoren entfalten ihren wahren Wert erst in Kombination mit anderen Transformations-, Kombinations- und Utility-Operatoren.
Zum Beispiel ist die Kombination mit `switchMap` oder `catchError` für "API-Umschaltung und Wiederherstellungsverarbeitung" ein häufiges Muster.

Für praktischere Anwendungsbeispiele siehe [Praktische Anwendungsfälle](./practical-use-cases.md) für detaillierte Erklärungen.
