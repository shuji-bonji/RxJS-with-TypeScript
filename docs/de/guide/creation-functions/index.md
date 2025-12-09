---
description: "RxJS-Erstellungsfunktionen (Observable-Erstellungsfunktionen) in RxJS, einschließlich der Unterschiede zum Pipeable Operator, der grundlegenden Verwendung und sieben Kategorien (grundlegende Erstellung, Schleifenbildung, HTTP-Kommunikation, Join, Auswahl und Teilung, bedingte Verzweigung, Steuerung). Systematische Erläuterung. Bietet eine Auswahlhilfe für die Eigenschaften und Verwendungen jeder Funktion."
---

# Creation Functions

In RxJS gibt es zwei verschiedene Formen von **Creation Functions** zum Erstellen von Observables und **Pipeable Operators** zum Konvertieren bestehender Observables.

Diese Seite beschreibt die grundlegenden Konzepte von Erstellungsfunktionen und die sieben Hauptkategorien.

## Was sind Erstellungsfunktionen?

**Erstellungsfunktionen** sind Funktionen zur Erstellung neuer Observables.

```typescript
import { of, from, interval } from 'rxjs';

// Als Creation Function verwenden
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

Sie werden direkt aus dem Paket `rxjs` importiert und als Funktionen zum Erstellen von Observables aufgerufen.

## Unterschiede zum Pipeable Operator

Erstellungsfunktionen und Pipeable-Operatoren haben unterschiedliche Verwendungen und Einsatzmöglichkeiten. In der folgenden Tabelle finden Sie die Unterschiede zwischen ihnen.

| Merkmal | Creation Function | Pipeable Operator |
|------|-------------------|-------------------|
| **Verwendung** | Erstellt neue Observable | Konvertiert bestehende Observable |
| **Import von** | `rxjs` | `rxjs/operators` |
| **Verwendungsart** | Direkter Funktionsaufruf | Innerhalb von `.pipe()` verwenden |
| **Beispiel** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Beispiele für Erstellungsfunktionen

Erstellungsfunktionen werden verwendet, um mehrere Observables direkt zu kombinieren.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Als Creation Function verwenden
concat(obs1$, obs2$).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5, 6
```

### Beispiel für einen Pipeable Operator

Der Pipeable-Operator wird verwendet, um einen Transformationsprozess zu einem bestehenden Observable hinzuzufügen.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Als Pipeable Operator verwenden
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5, 6
```

## Kriterien für die Verwendung

Die Wahl zwischen Creation Function und Pipeable Operator wird durch folgende Kriterien bestimmt.

### Wann die Erstellungsfunktion verwendet werden sollte

Die Creation Function eignet sich, wenn Sie mehrere Observables auf der gleichen Ebene bedienen wollen oder wenn Sie eine Observable von Grund auf neu erstellen wollen.

- **Wenn mehrere Observables auf derselben Ebene kombiniert werden**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **Bei der Erstellung einer Observable von Grund auf**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Wann der Pipeable-Operator verwendet werden sollte

Der Pipeable-Operator eignet sich für das Hinzufügen einer Verarbeitung zu einer bestehenden Observable oder für die Verkettung mehrerer Operationen.

- **Beim Hinzufügen einer Verarbeitung zu einer bestehenden Observable**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **Bei der Verkettung mehrerer Operationen als Pipeline**

## Kategorie der Erstellungsfunktionen

In diesem Kapitel werden die Erstellungsfunktionen in sieben Kategorien unterteilt.

### Liste aller Kategorien

In der folgenden Tabelle sehen Sie alle Kategorien und die darin enthaltenen Funktionen. Klicken Sie auf jeden Funktionsnamen, um zur Detailseite zu gelangen.

| Kategorie | Beschreibung | Hauptfunktionen | Typische Anwendungsfälle |
|---------|------|-----------|-------------------|
| **[Grundlegende Erstellung](/de/guide/creation-functions/basic/)** | Die grundlegendsten und am häufigsten verwendeten Funktionen. Erstellen von Daten, Arrays, Ereignissen und zeitbasierten Observables | [of](/de/guide/creation-functions/basic/of), [from](/de/guide/creation-functions/basic/from), [fromEvent](/de/guide/creation-functions/basic/fromEvent), [interval](/de/guide/creation-functions/basic/interval), [timer](/de/guide/creation-functions/basic/timer) | Festwertprüfung, Streaming vorhandener Daten, DOM-Ereignisverarbeitung, Polling, verzögerte Ausführung |
| **[Schleifengenerierung](/de/guide/creation-functions/loop/)** | Schleifenverarbeitung wie for- und while-Anweisungen, ausgedrückt als Observable | [range](/de/guide/creation-functions/loop/range), [generate](/de/guide/creation-functions/loop/generate) | Generierung fortlaufender Zahlen, Stapelverarbeitung, komplexe Zustandsübergänge, mathematische Berechnungen |
| **[HTTP-Kommunikation](/de/guide/creation-functions/http-communication/)** | HTTP-Kommunikation, die als Observable behandelt wird | [ajax](/de/guide/creation-functions/http-communication/ajax), [fromFetch](/de/guide/creation-functions/http-communication/fromFetch) | XMLHttpRequest-basierte HTTP-Kommunikation, Fetch API-basierte HTTP-Kommunikation, REST API-Aufrufe |
| **[Kombination](/de/guide/creation-functions/combination/)** | Kombinieren mehrerer Observables zu einer. Der Zeitpunkt und die Reihenfolge der Ausgabe unterscheiden sich je nach Kombinationsmethode | [concat](/de/guide/creation-functions/combination/concat), [merge](/de/guide/creation-functions/combination/merge), [combineLatest](/de/guide/creation-functions/combination/combineLatest), [zip](/de/guide/creation-functions/combination/zip), [forkJoin](/de/guide/creation-functions/combination/forkJoin) | Schrittweise Verarbeitung, Integration mehrerer Ereignisse, Synchronisierung von Formulareingaben, Warten auf den Abschluss paralleler API-Aufrufe |
| **[Auswahl und Partitionierung](/de/guide/creation-functions/selection/)** | Auswahl einer von mehreren Observables oder Aufteilung einer Observable in mehrere | [race](/de/guide/creation-functions/selection/race), [partition](/de/guide/creation-functions/selection/partition) | Wettbewerb zwischen mehreren Datenquellen, Erfolg/Misserfolg-Verzweigungsprozess |
| **[Bedingte Verzweigung](/de/guide/creation-functions/conditional/)** | Auswahl einer Observable auf der Grundlage einer Bedingung oder dynamische Erstellung einer Observable, wenn sie abonniert wurde | [iif](/de/guide/creation-functions/conditional/iif), [defer](/de/guide/creation-functions/conditional/defer) | Prozessverzweigung auf der Grundlage des Anmeldestatus, dynamische Erstellung von Observables, träge Auswertung |
| **[Steuerung](/de/guide/creation-functions/control/)** | Steuerung der Ausführungszeit und Ressourcenverwaltung von Observables | [scheduled](/de/guide/creation-functions/control/scheduled), [using](/de/guide/creation-functions/control/using) | Steuerung der Ausführungszeit durch Scheduler, Verwaltung des Lebenszyklus von Ressourcen, Vermeidung von Speicherlecks |

> [!TIP]
> **Lernreihenfolge**
>
> Wir empfehlen, dass Anfänger in der folgenden Reihenfolge lernen:
> 1. **Grundlegende Erstellung** - die grundlegenden RxJS-Funktionen
> 2. **Kombination** - die Grundlagen des Umgangs mit mehreren Streams
> 3. **HTTP-Kommunikation** - praktische API-Integration
> 4. Andere Kategorien - Lernen nach Bedarf

## Korrespondenz mit Pipeable Operator

Viele Erstellungsfunktionen haben einen entsprechenden Pipeable Operator. Wenn sie in einer Pipeline verwendet werden, wird die Familie der Operatoren `~With` verwendet.

| Creation Function | Pipeable Operator | Anmerkungen |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/de/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/de/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/de/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/de/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/de/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> Seit RxJS 7 wurden Operatoren vom Typ `~With` wie **[concatWith](/de/guide/operators/combination/concatWith)**, **[mergeWith](/de/guide/operators/combination/mergeWith)**, **[zipWith](/de/guide/operators/combination/zipWith)**, **[combineLatestWith](/de/guide/operators/combination/combineLatestWith)** und **[raceWith](/de/guide/operators/combination/raceWith)** hinzugefügt und sind nun einfacher als Pipeable-Operatoren zu verwenden.

## Welchen sollte ich verwenden?

Die Wahl zwischen Erstellungsfunktion und Pipeable Operator hängt vom Kontext ab.

### Die Erstellungsfunktion wird empfohlen

Wenn mehrere Observables auf der gleichen Ebene manipuliert werden sollen, ist die Creation Function die einfachere Wahl.

```typescript
// ✅ Mehrere Observables auf derselben Ebene kombinieren
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Pipeable Operator empfohlen

Wenn Sie Operationen als Teil einer Pipeline hinzufügen, verwenden Sie den Pipeable Operator, um den Ablauf der Verarbeitung zu verdeutlichen.

```typescript
// ✅ Als Teil einer Pipeline kombinieren
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## Zusammenfassung

- **Creation Functions**: Funktionen zum Erstellen oder Kombinieren von Observables
- **Pipeable Operators**: Funktionen zur Umwandlung bestehender Observables
- Creation Functions fallen in sieben Kategorien:
  1. **Grundlegende Erstellung**: erstellen Daten, Arrays, Ereignisse und zeitbasierte Observables
  2. **Schleifengenerierung**: stellen iterative Prozesse als Observables dar
  3. **HTTP-Kommunikation**: Behandelt HTTP-Kommunikation als Observable
  4. **Kombination**: Kombiniert mehrere zu einem
  5. **Auswahl und Partitionierung**: Auswahl oder Teilung
  6. **Bedingte Verzweigung**: Dynamisch nach Bedingungen generieren
  7. **Steuerung**: Steuerung der Ausführungszeit und Ressourcenverwaltung
- Pipelinefähige Operatoren in der `~With`-Familie von Pipelines
- Jede Kategorie enthält mehrere Funktionen, die je nach Anwendung verwendet werden können

## Nächste Schritte

Um mehr über die einzelnen Kategorien zu erfahren, folgen Sie den Links unten.

1. **[Grundlegende Erstellungsfunktionen](/de/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[Schleifengenerierung Erstellungsfunktionen](/de/guide/creation-functions/loop/)** - range, generate
3. **[HTTP-Kommunikation Erstellungsfunktionen](/de/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[Kombination Erstellungsfunktionen](/de/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[Auswahl und Partitionierung Erstellungsfunktionen](/de/guide/creation-functions/selection/)** - race, partition
6. **[Bedingte Verzweigung Erstellungsfunktionen](/de/guide/creation-functions/conditional/)** - iif, defer
7. **[Steuerung Erstellungsfunktionen](/de/guide/creation-functions/control/)** - scheduled, using

Auf jeder Seite können Sie mehr über die Erstellungsfunktionen und praktische Beispiele erfahren.

## Referenz-Ressourcen

- [Offizielle RxJS-Dokumentation - Creation Functions](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Creation Operators](https://www.learnrxjs.io/learn-rxjs/operators/creation)
