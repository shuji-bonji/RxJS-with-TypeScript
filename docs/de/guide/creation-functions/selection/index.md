---
description: "Auswahl- und Partitionierungs-Creation Functions (race, partition) für konkurrierende Prozesse und Stream-Aufteilung mit TypeScript."
---

# Auswahl- und Partitionierungs-Creation Functions

Creation Functions zur Auswahl eines Observable aus mehreren oder zur Aufteilung eines Observable in mehrere basierend auf Bedingungen.

## Was sind Auswahl- und Partitionierungs-Creation Functions?

Im Gegensatz zu Kombinations-Creation Functions haben Auswahl- und Partitionierungs-Creation Functions folgende Rollen:

- **Auswahl**: Auswahl eines Observable aus mehreren, das bestimmte Bedingungen erfüllt
- **Partitionierung**: Aufteilung eines Observable in mehrere Observables basierend auf Bedingungen

Diese Operationen sind das Gegenteil von "mehrere zu einem kombinieren" oder arbeiten aus einer anderen Perspektive.

## Hauptsächliche Auswahl- und Partitionierungs-Creation Functions

| Function | Beschreibung | Anwendungsfall |
|----------|--------------|----------------|
| **[race](/de/guide/creation-functions/selection/race)** | Nimmt das erste, das emittiert | Wettbewerb zwischen mehreren Datenquellen |
| **[partition](/de/guide/creation-functions/selection/partition)** | Teilt in zwei basierend auf Bedingung | Verzweigung bei Erfolg/Fehler |

## Kriterien für die Verwendung

### race - Auswahl des schnellsten Observable

`race` abonniert mehrere Observables gleichzeitig und nimmt **das Observable, das zuerst einen Wert emittiert**. Nicht ausgewählte Observables werden automatisch abgemeldet (unsubscribed).

**Anwendungsfälle**:
- Schnellste Antwort von mehreren API-Endpunkten
- Timeout-Verarbeitung (eigentliche Verarbeitung vs. Timer)
- Wettbewerb zwischen Cache und tatsächlichem API-Aufruf

```typescript
import { race, timer } from 'rxjs';
import { mapTo } from 'rxjs';

// Nimmt die schnellste von mehreren Datenquellen
const fast$ = timer(1000).pipe(mapTo('Fast API'));
const slow$ = timer(3000).pipe(mapTo('Slow API'));

race(fast$, slow$).subscribe(console.log);
// Ausgabe: 'Fast API' (wird nach 1 Sekunde ausgegeben, slow$ wird abgebrochen)
```

### partition - Aufteilung nach Bedingung

`partition` teilt ein Observable basierend auf einer Prädikatfunktion in **zwei Observables** auf. Der Rückgabewert ist ein Array `[true-Fall, false-Fall]`.

**Anwendungsfälle**:
- Trennung von Erfolg und Fehler
- Trennung von geraden und ungeraden Zahlen
- Trennung von gültigen und ungültigen Daten

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// In gerade und ungerade aufteilen
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Gerade:', val));
// Ausgabe: Gerade: 2, Gerade: 4, Gerade: 6

odd$.subscribe(val => console.log('Ungerade:', val));
// Ausgabe: Ungerade: 1, Ungerade: 3, Ungerade: 5
```

## Konvertierung von Cold zu Hot

Wie in der obigen Tabelle gezeigt, **erzeugen alle Auswahl- und Partitionierungs-Creation Functions Cold Observables**. Bei jedem Abonnement wird eine unabhängige Ausführung gestartet.

Durch Verwendung von Multicasting-Operatoren (`share()`, `shareReplay()` usw.) können Cold Observables in Hot Observables umgewandelt werden.

### Praxisbeispiel: Gemeinsame Nutzung konkurrierender API-Requests

```typescript
import { race, timer } from 'rxjs';
import { mapTo, share } from 'rxjs';

// ❄️ Cold - Führt den Wettbewerb bei jedem Abonnement erneut aus
const coldRace$ = race(
  timer(1000).pipe(mapTo('Fast API')),
  timer(3000).pipe(mapTo('Slow API'))
);

coldRace$.subscribe(val => console.log('Abonnent 1:', val));
coldRace$.subscribe(val => console.log('Abonnent 2:', val));
// → Jeder Abonnent führt einen unabhängigen Wettbewerb aus (2 Wettbewerbe)

// 🔥 Hot - Teilt das Wettbewerbsergebnis zwischen Abonnenten
const hotRace$ = race(
  timer(1000).pipe(mapTo('Fast API')),
  timer(3000).pipe(mapTo('Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Abonnent 1:', val));
hotRace$.subscribe(val => console.log('Abonnent 2:', val));
// → Teilt das Ergebnis eines Wettbewerbs
```

> [!TIP]
> Für Details siehe [Basis-Creation Functions - Konvertierung von Cold zu Hot](/de/guide/creation-functions/basic/#konvertierung-von-cold-zu-hot).

## Entsprechung zu Pipeable Operators

Auch für Auswahl- und Partitionierungs-Creation Functions gibt es entsprechende Pipeable Operators.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Nicht in Pipeline verwendbar (nur Creation Function) |

> [!NOTE]
> Für `partition` gibt es keine Pipeable Operator-Version. Wenn eine Aufteilung erforderlich ist, verwenden Sie die Creation Function oder teilen Sie manuell mit `filter` zweimal auf.

## Nächste Schritte

Um die detaillierte Funktionsweise und praktische Beispiele für jede Creation Function zu lernen, klicken Sie auf die Links in der obigen Tabelle.

Durch das gemeinsame Studium von [Kombinations-Creation Functions](/de/guide/creation-functions/combination/) und [Bedingte-Creation Functions](/de/guide/creation-functions/conditional/) können Sie ein umfassendes Verständnis der Creation Functions erlangen.
