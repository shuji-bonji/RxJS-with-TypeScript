---
description: "Verwendung von RxJS-Kombinations-Operatoren (Pipeable Operators) zur Kombination mehrerer Observables. withLatestFrom, concatWith, mergeWith, zipWith, raceWith und mehr - mit Unterschieden zu Creation Functions und Auswahlrichtlinien nach Verwendungszweck."
---

# Kombinations-Operatoren (Pipeable Operators)

RxJS-Kombinations-Operatoren (Combination) sind leistungsstarke Werkzeuge, um mehrere Observables zu kombinieren und neue Streams zu erstellen.

> [!IMPORTANT]
> Diese Seite behandelt **Pipeable Operators (Form für Verwendung innerhalb von Pipelines)**.
>
> Für **Creation Functions (Form zur Erstellung eines neuen Observables aus mehreren Observables)** siehe
> [Kapitel 3 Creation Functions](/de/guide/creation-functions/).

## Creation Functions vs Pipeable Operators

Funktionen im Zusammenhang mit Kombinationen werden in zwei Formen bereitgestellt.

### Creation Functions (erklärt in Kapitel 3)

Nimmt mehrere Observables als Argumente und erstellt ein neues Observable.

```typescript
import { concat, merge, combineLatest, zip, race, forkJoin } from 'rxjs';

// Verwendung als Creation Function
const combined$ = concat(obs1$, obs2$, obs3$);
const merged$ = merge(source1$, source2$);
```

Details siehe [Creation Functions](/de/guide/creation-functions/).

### Pipeable Operators (auf dieser Seite erklärt)

Verwendung innerhalb von `.pipe()` für vorhandene Observables.

```typescript
import { concatWith, mergeWith, combineLatestWith } from 'rxjs';

// Verwendung als Pipeable Operator
const result$ = source$.pipe(
  map(x => x * 2),
  concatWith(other$),
  filter(x => x > 10)
);
```

## Pipeable Operators Liste

### ◾ Auf dieser Seite behandelte Operatoren

|Operator|Beschreibung|
|---|---|
|[withLatestFrom](./withLatestFrom)|Kombiniert die neuesten Werte anderer Streams entsprechend der Ausgabe des Haupt-Observables|
|[mergeAll](./mergeAll)|Flacht Higher-order Observables parallel ab|
|[concatAll](./concatAll)|Flacht Higher-order Observables sequenziell ab|
|[switchAll](./switchAll)|Wechselt zum neuesten Higher-order Observable|
|[exhaustAll](./exhaustAll)|Ignoriert neue Higher-order Observables während der Ausführung|
|[combineLatestAll](./combineLatestAll)|Kombiniert die neuesten Werte aller internen Observables|
|[zipAll](./zipAll)|Paart entsprechende Werte jedes internen Observables|

### ◾ Als Creation Functions bereitgestellt

Die folgenden werden hauptsächlich als Creation Functions verwendet (siehe [Kapitel 3](/de/guide/creation-functions/)).

|Function|Beschreibung|Pipeable-Version|
|---|---|---|
|[concat](/de/guide/creation-functions/combination/concat)|Kombiniert in Reihenfolge|`concatWith` (RxJS 7+)|
|[merge](/de/guide/creation-functions/combination/merge)|Parallele Kombination|`mergeWith` (RxJS 7+)|
|[combineLatest](/de/guide/creation-functions/combination/combineLatest)|Kombiniert neueste Werte|`combineLatestWith` (RxJS 7+)|
|[zip](/de/guide/creation-functions/combination/zip)|Paart entsprechende Werte|`zipWith` (RxJS 7+)|
|[race](/de/guide/creation-functions/selection/race)|Verwendet den schnellsten Stream|`raceWith` (RxJS 7+)|
|[forkJoin](/de/guide/creation-functions/combination/forkJoin)|Wartet auf Abschluss aller|(Keine Pipeable-Version)|

## Für weitere praktische Lernzwecke

Beispiele für reale Szenarien mit Kombinations-Operatoren werden in
[Praktische Anwendungsfälle](./practical-use-cases.md) ausführlich vorgestellt.
