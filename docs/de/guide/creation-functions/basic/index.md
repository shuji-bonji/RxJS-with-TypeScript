---
description: Beschreibt Erstellungsfunktionen für die grundlegende Erstellung von Observables unter Verwendung von of, from, fromEvent, interval und timer aus einer Vielzahl von Datenquellen wie Einzelwerten, Arrays, Promises, Events und Timern. Es kann mit TypeScript Typsicherheit implementiert werden und ist ein wichtiges Konzept, das RxJS zugrunde liegt.
---

# Grundlegende Erstellungsfunktionen

Die grundlegendsten und am häufigsten verwendeten Erstellungsfunktionen. Erstellen Sie auf einfache Weise Daten, Arrays, Ereignisse und zeitbasierte Observables.

## Was sind grundlegende Erstellungsfunktionen?

Grundlegende Erstellungsfunktionen sind Funktionen zur Erstellung einer einzelnen Observable aus verschiedenen Datenquellen. Sie sind der grundlegendste Satz von Funktionen für die Verwendung von RxJS und werden in fast allen RxJS-Codes verwendet.

In der folgenden Tabelle sind die Merkmale und die Verwendung der einzelnen Erstellungsfunktionen aufgeführt.

## Wichtigste grundlegende Erstellungsfunktionen

| Funktion | Beschreibung | Anwendungsfall |
|----------|------|-------------|
| **[of](/de/guide/creation-functions/basic/of)** | Gibt angegebene Werte nacheinander aus | Festwerttest, Mock-Erstellung |
| **[from](/de/guide/creation-functions/basic/from)** | Konvertiert aus Array, Promise, etc. | Streaming vorhandener Daten |
| **[fromEvent](/de/guide/creation-functions/basic/fromEvent)** | Konvertiert Ereignisse in Observable | DOM-Ereignisse, Node.js EventEmitter |
| **[interval](/de/guide/creation-functions/basic/interval)** | Kontinuierliche Ausgabe in regelmäßigen Abständen | Polling, periodische Ausführung |
| **[timer](/de/guide/creation-functions/basic/timer)** | Start nach einer Verzögerung | Verzögerte Ausführung, Timeout |

## Verwendungskriterien

Die Wahl der grundlegenden Erstellungsfunktionen wird durch die Art der Datenquelle bestimmt.

### 1. Datentyp

- **Statische Werte**: `of()` - erzeugt ein Observable durch direkte Angabe des Wertes
- **Arrays und Iterables**: `from()` - Konvertiert eine bestehende Sammlung in einen Stream
- **Promise**: `from()` - konvertiert eine asynchrone Verarbeitung in ein Observable
- **Event**: `fromEvent()` - konvertiert einen Ereignis-Listener in ein Observable
- **Zeitbasiert**: `interval()`, `timer()` - Veröffentlichen von Werten basierend auf dem Ablauf der Zeit

### 2. Zeitpunkt der Ausgabe

- **Sofort**: `of()`, `from()` - Beginn der Veröffentlichung von Werten, sobald sie abonniert werden
- **Wenn ein Ereignis eintritt**: `fromEvent()` - Veröffentlichen, sobald ein Ereignis eintritt
- **Periodisch ausgegeben**: `interval()` - kontinuierlich in regelmäßigen Abständen ausgegeben
- **Veröffentlichung nach einer Verzögerung**: `timer()` - Beginn der Veröffentlichung nach einer bestimmten Zeit

### 3. Zeitpunkt der Beendigung

- **Sofortige Beendigung**: `of()`, `from()` - Beendigung, nachdem alle Werte ausgegeben worden sind
- **Nicht abgeschlossen**: `fromEvent()`, `interval()` - Fortsetzung bis zur Abmeldung
- **Abgeschlossen nach einmaliger Ausgabe**: `timer(delay)` - Abschluss nach Ausgabe eines Wertes

## Praktische Anwendungsfälle

### of() - Prüfung mit festen Werten

```typescript
import { of } from 'rxjs';

// Testdaten erstellen
const mockUser$ = of({ id: 1, name: 'Test User' });

mockUser$.subscribe(user => console.log(user));
// Ausgabe: { id: 1, name: 'Test User' }
```

### from() - Streaming eines Arrays

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// Ausgabe: 2, 4, 6, 8, 10
```

### fromEvent() - Klick-Ereignis

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Button clicked!'));
```

### interval() - Polling

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// Fragt die API alle 5 Sekunden ab
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Updated:', data));
```

### timer() - verzögerte Ausführung

```typescript
import { timer } from 'rxjs';

// Führt nach 3 Sekunden aus
timer(3000).subscribe(() => console.log('3 seconds passed'));
```

## Vorsicht vor Speicherlecks

Bei der Verwendung der grundlegenden Erstellungsfunktionen ist eine ordnungsgemäße Abmeldung wichtig.

> [!WARNING]
> `fromEvent()`, `interval()` und periodische `timer(delay, period)` müssen immer `unsubscribe()` oder automatisch mit `takeUntil()` usw. abbestellt werden, wenn die Komponente zerstört wird, da sie nicht abgeschlossen werden.
>
> Hinweis: Wenn das zweite Argument weggelassen wird, wie in `timer(delay)`, wird es automatisch nach einer Ausgabe beendet.

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Window resized'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Umwandlung von Cold zu Hot

Wie in der obigen Tabelle gezeigt, erzeugen **alle grundlegenden Erstellungsfunktionen Cold Observable**. Jedes Abonnement initiiert eine unabhängige Ausführung.

Allerdings kann ein **Cold Observable in ein Hot Observable umgewandelt** werden, indem die folgenden Multicast-Operatoren verwendet werden.

### Bedingungen und Operatoren für die Umwandlung in Hot

| Operator | Verhalten | Anwendungsfall |
|-------------|------|-------------|
| **share()** | Multicast + automatische Verbindung/Trennung | HTTP-Anfrage mit mehreren Abonnenten teilen |
| **shareReplay(n)** | Die letzten n Werte zwischenspeichern und an neue Abonnenten liefern | API-Antworten zwischenspeichern |
| **publish() + connect()** | Multicast manuell starten | Ausführen, wenn alle Abonnenten vorhanden sind |
| **multicast(subject)** | Multicast mit benutzerdefiniertem Subject | Wenn erweiterte Kontrolle erforderlich ist |

### Praktische Beispiele

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - Unabhängiger Timer pro Abonnement
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// Ausgabe:
// A: 0 (nach 0 Sekunden)
// A: 1 (nach 1 Sekunde)
// B: 0 (nach 1,5 Sekunden) ← B startet unabhängig bei 0
// A: 2 (nach 2 Sekunden)
// B: 1 (nach 2,5 Sekunden)

// 🔥 Hot - Timer wird zwischen den Abonnenten geteilt
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// Ausgabe:
// A: 0 (nach 0 Sekunden)
// A: 1 (nach 1 Sekunde)
// A: 2, B: 2 (nach 2 Sekunden) ← B kommt dazu und erhält den gleichen Wert
```

> [!TIP]
> **Fälle, in denen Hot erforderlich ist**:
> - Ich möchte eine HTTP-Anfrage mit mehreren Abonnenten teilen
> - Ich möchte nur eine WebSocket- oder Server-Verbindung unterhalten
> - Ich möchte die Ergebnisse einer kostenintensiven Berechnung an mehreren Stellen verwenden
>
> Weitere Informationen finden Sie im Kapitel **Subject und Multicast** (Kapitel 5).

## Beziehung zu Pipeable Operator

Die grundlegenden Erstellungsfunktionen haben keinen direkten Pipeable Operator als Gegenstück. Sie werden immer als Erstellungsfunktionen verwendet.

Sie werden jedoch in Kombination mit Pipeable-Operatoren nach dem folgenden Muster verwendet:

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// Benutzereingabe → 300 ms Wartezeit → API-Aufruf
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## Nächste Schritte

Wenn Sie mehr über die Funktionsweise der einzelnen Erstellungsfunktionen und praktische Beispiele erfahren möchten, klicken Sie auf die Links in der Tabelle oben.

Sie können auch die [Kombinationsfunktionen](/de/guide/creation-functions/combination/), [Auswahl- und Partitionierungsfunktionen](/de/guide/creation-functions/selection/) und [bedingte Verzweigungsfunktionen](/de/guide/creation-functions/conditional/) zusammen studieren, um ein umfassenderes Bild der Erstellungsfunktionen zu erhalten.
