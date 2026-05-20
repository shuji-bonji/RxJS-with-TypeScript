---
description: "Creation Function (race und partition), die aus mehreren Observable eines auswählen oder ein Observable in mehrere aufteilen, werden erläutert. Es werden praktische Anwendungsfälle und typsichere Implementierungen in TypeScript vorgestellt, z. B. Konfliktbehandlung, schnellste Reaktionserfassung und Stream-Partitionierung durch bedingte Verzweigung."
---

# Auswahl und Aufteilung der Systeme Creation Function

Creation Function, die ein Observable aus mehreren Observables auswählen oder ein Observable entsprechend den Bedingungen in mehrere Observables aufteilen.

## Auswahl- und Aufteilungssystem Was sind Creation Function?

Creation Functions für Selektions- und Splitting-Systeme unterscheiden sich von denen für kombinierende Systeme und haben die folgenden Rollen.

- **Auswahl**: wählt unter mehreren Observables ein Observable aus, das bestimmte Bedingungen erfüllt.
- **Aufteilung**: Aufteilung eines Observables in mehrere Observables gemäß einer Bedingung.

Diese Funktionen wirken in die entgegengesetzte Richtung oder aus einer anderen Perspektive als die Verknüpfung "mehrere zu einer".

## Hauptauswahl- und Aufteilungssysteme Creation Function

| Funktion | Beschreibung | Anwendungsfall. |
|---|---|---|
| **[race](/de/guide/creation-functions/selection/race)** | Übernahme des zuerst veröffentlichten | Wettbewerb mit mehreren Datenquellen |
| **[partition](/de/guide/creation-functions/selection/partition)** | Zweiteilung mit Bedingungen | Prozess für Erfolg/Misserfolg aufteilen |

## Kriterien für die Verwendung

### race - wählen Sie das schnellste Observable

race" abonniert mehrere Observables gleichzeitig und nimmt das **erste Observable an, das einen Wert liefert. Observables, die nicht übernommen werden, werden automatisch unsubscribed.

**Verwendungsfall**:.
- Übernahme der schnellsten Antwort von mehreren API-Endpunkten
- Behandlung von Zeitüberschreitungen (ursprünglicher Prozess vs. Zeitgeber)
- Wettbewerb zwischen Cache und tatsächlichen API-Aufrufen

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Übernehmen Sie das Schnellste aus mehreren Datenquellen
const fast$ = timer(1000).pipe(map(() => 'Fast API'));
const slow$ = timer(3000).pipe(map(() => 'Slow API'));

race(fast$, slow$).subscribe(console.log);
// Ausgabe.: 'Fast API' (1Die Ausgabe wird nach einer Sekunde ausgegeben,slow$wird abgebrochen)
```

### partition - aufgeteilt nach Bedingung

Die `partition` teilt ein Observable in **zwei Observables** basierend auf einer bedingten Funktion. Der Rückgabewert ist ein Array `[if true, if false]`.

**Gebrauchsfall**:.
- Trennung von Erfolg und Misserfolg.
- Unterscheidung von geraden und ungeraden Zahlen.
- Unterscheidung von gültigen und ungültigen Daten

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Aufteilung in gerade und ungerade Zahlen
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// Ausgabe.: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// Ausgabe.: Odd: 1, Odd: 3, Odd: 5
```

## Konvertierung von kalt nach heiß

Wie in der obigen Tabelle dargestellt, erzeugen **Alle Selection und Split System Creation Function ein Cold Observable. Jedes Abonnement initiiert eine unabhängige Ausführung.

Cold Observable können mit Hilfe von Multicast-basierten Operatoren (`share()`, `shareReplay()`, usw.) in Hot Observable umgewandelt werden.

### Praktisches Beispiel: Gemeinsame Nutzung widersprüchlicher API-Anfragen

```typescript
import { race, timer } from 'rxjs';
import { map, share } from 'rxjs';

// ❄️ Cold - Wiederholung des Wettbewerbs für jedes Abonnement
const coldRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
);

coldRace$.subscribe(val => console.log('Teilnehmer1:', val));
coldRace$.subscribe(val => console.log('Teilnehmer2:', val));
// → Jeder Teilnehmer führt ein eigenständiges Gewinnspiel durch (ein2Wettbewerb einmal)

// 🔥 Hot - Austausch von Wettbewerbsergebnissen zwischen Abonnenten
const hotRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Teilnehmer1:', val));
hotRace$.subscribe(val => console.log('Teilnehmer2:', val));
// → 1Gemeinsame Nutzung der Ergebnisse eines Wettbewerbs
```

> [!TIP]

> Weitere Informationen finden Sie unter [Grundlegendes Erstellungssystem - Umwandlung von kalt nach heiß](/de/guide/creation-functions/basic/#cold- to -hot-).

## Korrespondenz mit Pipeable Operator

Die Creation Functions für Selektion und Division haben auch einen entsprechenden Pipeable Operator.

| Creation Function | Pipeable Operator |
|---|---|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| partition(source$, predicate)` | Kann nicht innerhalb einer Pipeline verwendet werden (nur Creation Function). |

> [!NOTE]

> Es gibt keine Pipeable Operator Version von `partition`. Wenn eine Partition benötigt wird, kann sie als Creation Function verwendet oder manuell mit `filter` zweimal geteilt werden.

## Nächste Schritte.

Um mehr über die detaillierte Funktionsweise und die praktischen Beispiele der einzelnen Creation Functions zu erfahren, klicken Sie auf die Links in der obigen Tabelle.

Sie können auch [Kombinierte Creation Function](/de/guide/creation-functions/combination/) und [Conditional Creation Function](/de/guide/creation-functions/conditional/) lernen. Zusammen vermitteln sie ein ganzheitliches Verständnis der Creation Function.
