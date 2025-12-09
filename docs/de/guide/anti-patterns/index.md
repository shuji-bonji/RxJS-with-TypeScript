---
description: "Ein praktischer Leitfaden zum Verständnis von RxJS-Anti-Patterns und zum Schreiben von robusterem und wartbarem Code, der systematisch die Probleme und Lösungen erklärt, die bei der Entwicklung häufig auftreten, wie z. B. Missbrauch von Subjects, verschachtelte Subscriptions, bedingte Verzweigungen innerhalb von Subscriptions und Flag-Wildwuchs."
---

# RxJS Anti-Patterns Sammlung

RxJS ist eine leistungsstarke Bibliothek für reaktive Programmierung, aber bei falscher Verwendung kann sie eine Brutstätte für Fehler und reduzierte Wartbarkeit sein. In diesem Abschnitt stellen wir häufige Fehler bei der Verwendung von RxJS in TypeScript und Best Practices zur Vermeidung dieser Fehler vor.

## Zweck dieses Abschnitts

- **Bugs verhindern**: Vermeiden Sie Implementierungsprobleme, indem Sie häufige Fehler im Voraus verstehen
- **Wartbarkeit verbessern**: Lernen Sie Codemuster, die einfach zu lesen und zu testen sind
- **Leistungsoptimierung**: Erlernen von Techniken zur Vermeidung von Speicherlecks und unnötiger Verarbeitung

## Liste der Anti-Patterns

Dieser Abschnitt behandelt die folgenden 17 Anti-Patterns.

### 🔴 Kritische Probleme

Diese Muster können schwerwiegende Auswirkungen auf Ihre Anwendung haben.

| Muster | Problem | Auswirkung |
|---|---|---|
| **[Externe Veröffentlichung des Subjects](./common-mistakes#1-subject-externe-veröffentlichung)** | `Subject` direkt exponieren und externe Aufrufe von `next()` erlauben | Unvorhersehbarkeit der Zustandsverwaltung, Debugging-Schwierigkeiten |
| **[Verschachteltes subscribe](./common-mistakes#2-verschachteltes-subscribe-callback-hölle)** | `subscribe` innerhalb von `subscribe` aufrufen | Callback-Hölle, Komplikationen bei der Fehlerbehandlung |
| **[Zustandsverwaltungs-Flag-Wildwuchs](./flag-management)** | 17 boolesche Flags zur Zustandsverwaltung, Rest des imperativen Denkens | Reduzierte Lesbarkeit, schwierige Wartung, Brutstätte von Bugs |
| **[if-Verschachtelung in subscribe](./subscribe-if-hell)** | Komplexe bedingte Verzweigungen innerhalb von `subscribe` (3+ Verschachtelungen) | Reduzierte Lesbarkeit, schwer zu testen, verletzt deklaratives Denken |
| **[unsubscribe vergessen](./common-mistakes#3-unsubscribe-vergessen-speicherleck)** | Unendliche Streams nicht abbestellen | Speicherleck, Ressourcenverschwendung |
| **[Missbrauch von shareReplay](./common-mistakes#4-sharereplay-missbrauch)** | `shareReplay` ohne Verständnis der Funktionsweise verwenden | Veraltete Datenreferenzen, Speicherlecks |

### 🟡 Probleme, die Aufmerksamkeit erfordern

Diese können in bestimmten Situationen ein Problem darstellen.

| Muster | Problem | Auswirkungen |
|---|---|---|
| **[Nebeneffekte in map](./common-mistakes#5-nebeneffekte-in-map)** | Zustandsänderung im `map`-Operator | Unvorhersehbares Verhalten, schwer zu testen |
| **[Cold/Hot ignorieren](./common-mistakes#6-cold-hot-observable-unterschiede-ignorieren)** | Observable-Natur nicht berücksichtigen | Doppelte Ausführung, unerwartetes Verhalten |
| **[Vermischung mit Promise](./promise-observable-mixing)** | Fehlerhafte Konvertierung zwischen Promise und Observable | Nicht abbrechbar, schlechte Fehlerbehandlung |
| **[Backpressure ignorieren](./common-mistakes#8-backpressure-ignorieren)** | Fehler bei der Kontrolle hochfrequenter Ereignisse | Leistungseinbußen, Einfrieren der UI |

### 🔵 Probleme mit der Codequalität

Hierbei handelt es sich nicht um direkte Fehler, sondern um Faktoren, die die Codequalität beeinträchtigen.

| Muster | Problem | Auswirkung |
|---|---|---|
| **[Fehler verschlucken](./common-mistakes#9-fehler-verschlucken)** | Fehler werden nicht richtig behandelt | Schwierigkeiten bei der Fehlersuche, schlechte Benutzererfahrung |
| **[DOM-Ereignisleck](./common-mistakes#10-dom-ereignis-subscription-leck)** | Keine Freigabe von DOM-Ereignis-Listenern | Speicherlecks, schlechte Leistung |
| **[Fehlende Typsicherheit](./common-mistakes#11-typsicherheit-fehlt-any-übernutzung)** | Starke Verwendung von `any` | Laufzeitfehler, Schwierigkeiten beim Refactoring |
| **[Falsche Operatorauswahl](./common-mistakes#12-falsche-operatorauswahl)** | Verwendung von Operatoren, die für den Zweck nicht geeignet sind | Ineffizienz, unerwartetes Verhalten |
| **[Übermäßige Komplexität](./common-mistakes#13-übermäßige-komplexität)** | Verkomplizierung von Prozessen, die einfach geschrieben werden könnten | Reduzierte Lesbarkeit, schwer zu pflegen |
| **[Einzeiler-Hölle](./one-liner-hell)** | Mischung aus Stream-Definitionen, Transformationen und Subscriptions | Schwierig zu debuggen, schwierig zu testen, reduzierte Lesbarkeit |
| **[Zustandsänderungen in subscribe](./common-mistakes#14-zustandsänderung-in-subscribe)** | Direkte Zustandsänderung innerhalb von `subscribe` | Schwierig zu testen, verursacht Bugs |
| **[Mangel an Tests](./common-mistakes#15-mangel-an-tests)** | Keine Tests für RxJS-Code schreiben | Regression, Refactoring-Schwierigkeiten |

## Lernprozess

1. **[Häufige Fehler und wie man mit ihnen umgeht](./common-mistakes)** um 15 Anti-Patterns im Detail zu lernen
2. Für jedes Anti-Pattern finden Sie ein "schlechtes Beispiel" und ein "gutes Beispiel" Code
3. **[Checkliste zur Vermeidung von Anti-Patterns](./checklist)**, um Ihren Code zu überprüfen
4. Implementieren Sie Best Practices und teilen Sie diese mit Ihrem Team

## Verwandte Abschnitte

Nachdem Sie sich über Anti-Patterns informiert haben, sollten Sie auch die folgenden Abschnitte lesen:

- **[Fehlerbehandlung](/de/guide/error-handling/strategies)** - Geeignete Strategien zur Fehlerbehandlung
- **[Testmethoden](/de/guide/testing/unit-tests)** - Wie man RxJS-Code testet
- **[Verständnis von Operatoren](/de/guide/operators/)** - Wie man den richtigen Operator auswählt

## Nächste Schritte

1. Beginnen Sie mit **[Häufige Fehler und wie man damit umgeht](./common-mistakes)**, um praktische Anti-Patterns und ihre Lösungen zu lernen.
2. Verwenden Sie nach dem Lernen die **[Checkliste zur Vermeidung von Anti-Patterns](./checklist)**, um den aktuellen Code zu überprüfen.

---

**WICHTIG**: Diese Anti-Patterns sind häufig in realen Projekten zu finden. Sie frühzeitig zu verstehen, wird Ihnen helfen, hochwertigen RxJS-Code zu schreiben.
