---
description: "Beschreibt Erstellungsfunktionen, die mehrere Observables zu einer einzigen kombinieren; den Unterschied zwischen und die Verwendung von concat, merge, combineLatest, zip, forkJoin und race; Anwendungsfälle für jede Funktion und typsichere Implementierungsmuster in TypeScript mit praktischen Codebeispielen."
---

# Kombinationsfunktionen

Die wichtigsten Erstellungsfunktionen zum Kombinieren mehrerer Observables zu einem einzigen Observable.

## Was sind Kombinationsfunktionen?

Die Kombinationsfunktionen nehmen mehrere Observables und kombinieren sie zu einem einzigen Observable-Stream. Je nach Kombinationsmethode unterscheiden sich der Zeitpunkt und die Reihenfolge, in der die Werte ausgegeben werden.

In der nachstehenden Tabelle finden Sie die Merkmale und die Verwendung der einzelnen Erstellungsfunktionen.

## Hauptkombinationsfunktionen

| Funktion | Beschreibung | Anwendungsfall |
|----------|------|-------------|
| **[concat](/de/guide/creation-functions/combination/concat)** | Sequenzielle Kombination (nächste beginnt nach Abschluss der vorherigen) | Schrittweise Verarbeitung |
| **[merge](/de/guide/creation-functions/combination/merge)** | Gleichzeitige Kombination (gleichzeitiges Abonnement, Ausgabe in Veröffentlichungsreihenfolge) | Zusammenführen mehrerer Ereignisse |
| **[combineLatest](/de/guide/creation-functions/combination/combineLatest)** | Neueste Werte kombinieren | Formulareingabe synchronisieren |
| **[zip](/de/guide/creation-functions/combination/zip)** | Entsprechende Werte paaren | Entsprechung zwischen Anfrage und Antwort |
| **[forkJoin](/de/guide/creation-functions/combination/forkJoin)** | Endwerte kombinieren und auf alle Abschlüsse warten | Auf Abschluss paralleler API-Aufrufe warten |

## Verwendungskriterien

Die Wahl der Kombinationsfunktionen wird durch folgende Gesichtspunkte bestimmt.

### 1. Ausführungszeitpunkt

- **Sequentielle Ausführung**: `concat` - startet die nächste, nachdem die vorherige Observable abgeschlossen ist
- **Parallele Ausführung**: `merge`, `combineLatest`, `zip`, `forkJoin` - alle Observables gleichzeitig abonnieren

### 2. Art der Wertausgabe

- **Alle Werte ausgeben**: `concat`, `merge` - alle von jedem Observable ausgegebenen Werte ausgeben
- **Neueste Werte kombinieren**: `combineLatest` - jedes Mal wenn eine Observable einen Wert ausgibt, werden alle neuesten Werte kombiniert
- **Entsprechende Werte paaren**: `zip` - Paare bilden und den Wert an der entsprechenden Position in jedem Observable ausgeben
- **Nur Endwerte**: `forkJoin` - gibt jeden Endwert als Array aus, wenn alle Observables abgeschlossen sind

### 3. Zeitpunkt des Abschlusses

- **Nach allen Abschlüssen**: `concat`, `forkJoin` - warten, bis alle Observables abgeschlossen sind
- **Abgeschlossen beim kürzesten Stream**: `zip` - abgeschlossen, wenn eine abgeschlossen ist, da die verbleibenden Werte nicht mehr gepaart werden können
- **Nicht abgeschlossen**: `merge`, `combineLatest` - nicht abgeschlossen, wenn eine abgeschlossen ist, während die andere weiterläuft

## Umwandlung von Cold zu Hot

Wie aus der obigen Tabelle hervorgeht, erzeugen **alle Kombinationsfunktionen Cold Observable**. Jedes Abonnement initiiert eine unabhängige Ausführung.

Allerdings können **Cold Observables in Hot Observables umgewandelt** werden, indem Multicast-Operatoren (wie `share()`, `shareReplay()`, `publish()` usw.) verwendet werden.

### Praktisches Beispiel: Gemeinsame Nutzung von HTTP-Anfragen

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - Unabhängige HTTP-Anfrage pro Abonnement
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
);

coldApi$.subscribe(val => console.log('Abonnent 1:', val));
coldApi$.subscribe(val => console.log('Abonnent 2:', val));
// → Jeder Abonnent führt unabhängiges interval aus (doppelte Anfragen)

// 🔥 Hot - Gemeinsame Ausführung zwischen Abonnenten
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('Abonnent 1:', val));
hotApi$.subscribe(val => console.log('Abonnent 2:', val));
// → Ein gemeinsames interval (nur eine Anfrage)
```

> [!TIP]
> **Fälle, in denen Hot erforderlich ist**:
> - Mehrere Komponenten teilen die gleichen API-Ergebnisse
> - Mehrere Stellen verwenden das Ergebnis einer parallelen Anfrage mit `forkJoin`
> - Status mit `combineLatest` verwalten und an mehrere Abonnenten verteilen
>
> Weitere Informationen finden Sie unter [Grundlegende Erstellungsfunktionen - Umwandlung von Cold zu Hot](/de/guide/creation-functions/basic/#umwandlung-von-cold-zu-hot).

## Korrespondenz mit Pipeable Operator

Für die Kombinationsfunktionen gibt es entsprechende Pipeable Operatoren. Bei Verwendung in einer Pipeline wird der `~With`-Operator verwendet.

| Erstellungsfunktion | Pipeable Operator |
|-------------------|-------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## Nächste Schritte

Um mehr über die detaillierte Funktionsweise und praktische Beispiele der einzelnen Erstellungsfunktionen zu erfahren, klicken Sie auf die Links in der obigen Tabelle.

Sie können auch [Auswahl- und Partitionierungsfunktionen](/de/guide/creation-functions/selection/) und [Bedingte Verzweigungsfunktionen](/de/guide/creation-functions/conditional/) zusammen erlernen, um einen Gesamtüberblick über die Erstellungsfunktionen zu erhalten.
