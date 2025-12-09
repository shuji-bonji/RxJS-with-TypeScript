---
description: "RxJS-Debugging-Techniken: Wertverfolgung mit tap(), effektive Platzierung von console.log, RxJS DevTools-Erweiterung, Erstellung benutzerdefinierter Debug-Operatoren und Leistungsmessung. Systematische Erklärung praktischer Debugging-Strategien und Methoden zur Identifikation von Problemen mit ausbleibenden Werten."
---

# RxJS-Debugging-Techniken

Das Debugging von RxJS erfordert aufgrund der asynchronen Natur von Streams einen anderen Ansatz als herkömmliche synchrone Debugging-Methoden.

Diese Seite bietet grundlegende Strategien zum Debuggen von RxJS-Anwendungen und Navigation zu detaillierten Debugging-Techniken.

## Übersicht der Debugging-Techniken

Das Debugging von RxJS lässt sich in die folgenden 4 Ansätze kategorisieren:

| Ansatz | Inhalt | Detailseite |
|----------|------|-----------|
| **Grundstrategie** | tap-Operator, Entwicklerwerkzeuge, RxJS DevTools | Auf dieser Seite erklärt |
| **Häufige Szenarien** | 6 typische Probleme wie keine Werte, Memory Leaks, übersehene Fehler | [→ Details](/de/guide/debugging/common-scenarios) |
| **Benutzerdefinierte Tools** | Benannte Streams, Debug-Operatoren, Leistungsmessung | [→ Details](/de/guide/debugging/custom-tools) |
| **Performance** | Überwachung der Subscription-Anzahl, Erkennung von Neuberechnungen, Speichernutzung, Best Practices | [→ Details](/de/guide/debugging/performance) |

## Grundlegende Debugging-Strategien

### 1. Log-Ausgabe mit dem `tap`-Operator

Der `tap`-Operator ist die grundlegendste Debugging-Technik, mit der Sie Stream-Werte ohne Nebenwirkungen beobachten können.

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 Originalwert:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 Nach map:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 Nach filter:', value))
  )
  .subscribe(value => console.log('✅ Endwert:', value));

// Ausgabe:
// 🔵 Originalwert: 0
// 🟢 Nach map: 0
// 🔵 Originalwert: 1
// 🟢 Nach map: 2
// 🔵 Originalwert: 2
// 🟢 Nach map: 4
// 🔵 Originalwert: 3
// 🟢 Nach map: 6
// 🟡 Nach filter: 6
// ✅ Endwert: 6
```

#### Wichtige Punkte
- Durch Einfügen von `tap` in jeden Schritt der Pipeline können Sie den Datenfluss verfolgen
- Die Verwendung von Emojis und Labels verbessert die Sichtbarkeit der Logs
- `tap` ändert keine Werte, sodass Debug-Logs sicher eingefügt werden können

### 2. Ausgabe detaillierter Log-Informationen

Für detailliertere Debug-Informationen verwenden Sie ein Observer-Objekt.

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// Normaler Stream
of(1, 2, 3)
  .pipe(debug('Normal'))
  .subscribe();

// Ausgabe:
// [Normal] next: 1
// [Normal] next: 2
// [Normal] next: 3
// [Normal] complete

// Stream mit Fehler
concat(
  of(1, 2),
  throwError(() => new Error('Fehler aufgetreten'))
)
  .pipe(debug('Fehler'))
  .subscribe({
    error: () => {} // Fehlerbehandlung
  });

// Ausgabe:
// [Fehler] next: 1
// [Fehler] next: 2
// [Fehler] error: Error: Fehler aufgetreten
```

### 3. Überprüfung mit Entwicklerwerkzeugen

Debugging-Techniken mit Browser-Entwicklerwerkzeugen.

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// Debug-Hilfsfunktion
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Value:', value);
      console.log('Type:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// Debugging von Button-Click-Events
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 Senden:', data));
}
```

#### Nutzung der Entwicklerwerkzeuge
- Gruppierung von Logs mit `console.group()`
- Anzeige von Stack-Traces mit `console.trace()`
- Übersichtliche Darstellung von Arrays und Objekten mit `console.table()`
- Setzen von Breakpoints innerhalb von `tap`

### 4. Verwendung von RxJS DevTools

RxJS DevTools ist ein als Browser-Erweiterung verfügbares Debugging-Tool.

#### Installation
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### Hauptfunktionen
- Visualisierung des Subscription-Status von Observables
- Timeline-Anzeige von Stream-Werten
- Erkennung von Memory Leaks
- Performance-Analyse

#### Verwendungsbeispiel

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// Debugging nur in der Entwicklungsumgebung aktivieren
// Die Methode zur Überprüfung von Umgebungsvariablen hängt vom Build-Tool ab
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // Manuelle Einstellung: Verwendung einer globalen Variable
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // Für DevTools beobachtbar machen
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## Detaillierte Debugging-Techniken

Nachdem Sie die Grundstrategien verstanden haben, lernen Sie auf den folgenden Detailseiten spezifische Debugging-Techniken.

### Häufige Debugging-Szenarien

6 typische Probleme, die in der tatsächlichen Entwicklung auftreten, und deren Lösungen

- Szenario 1: Keine Werte fließen
- Szenario 2: Unerwartete Werte werden ausgegeben
- Szenario 3: Subscription wird nicht abgeschlossen (unendlicher Stream)
- Szenario 4: Memory Leak (vergessenes Unsubscribe)
- Szenario 5: Fehler werden nicht bemerkt
- Szenario 6: Verfolgung der Anzahl von Retry-Versuchen

[→ Häufige Debugging-Szenarien anzeigen](/de/guide/debugging/common-scenarios)

### Benutzerdefinierte Debug-Tools

Erstellung eigener Debug-Tools entsprechend den Projektanforderungen

- Debugging benannter Streams (tagStream)
- Erstellung benutzerdefinierter Debug-Operatoren
- Performance-Mess-Operatoren (measure)

[→ Benutzerdefinierte Debug-Tools anzeigen](/de/guide/debugging/custom-tools)

### Performance-Debugging

Anwendungsoptimierung und Best Practices

- Überprüfung und Verfolgung der Subscription-Anzahl
- Erkennung unnötiger Neuberechnungen (shareReplay)
- Überwachung der Speichernutzung
- Aufbau einer Debug-Umgebung
- Typsicheres Debugging
- Einrichtung von Error Boundaries

[→ Performance-Debugging anzeigen](/de/guide/debugging/performance)

## Zusammenfassung

RxJS-Debugging kann effizient durchgeführt werden, indem die folgenden Punkte beachtet werden.

### Grundstrategie
- ✅ Beobachtung jeder Phase des Streams mit dem `tap`-Operator
- ✅ Detaillierte Log-Ausgabe mit Entwicklerwerkzeugen
- ✅ Visualisierung von Streams mit RxJS DevTools

### Häufige Szenarien
- ✅ Keine Werte fließen → Überprüfung von vergessenen Subscriptions, Filterbedingungen
- ✅ Unerwartete Werte → Beachten Sie die Reihenfolge der Operatoren und gemeinsame Referenzen
- ✅ Subscription wird nicht abgeschlossen → Verwendung von `take` oder `takeUntil` für unendliche Streams
- ✅ Memory Leak → Automatisches Unsubscribe mit `takeUntil`-Pattern
- ✅ Übersehene Fehler → Implementierung angemessener Fehlerbehandlung

### Debug-Tools
- ✅ Flexibles Debugging mit benutzerdefinierten Debug-Operatoren
- ✅ Verfolgung mehrerer Streams mit benannten Streams
- ✅ Identifikation von Bottlenecks durch Leistungsmessung

### Performance
- ✅ Vermeidung von Memory Leaks durch Überwachung der Subscription-Anzahl
- ✅ Vermeidung unnötiger Neuberechnungen mit `shareReplay`
- ✅ Regelmäßige Überprüfung der Speichernutzung

Durch die Kombination dieser Techniken können Sie RxJS-Anwendungen effizient debuggen.

## Verwandte Seiten

- [Fehlerbehandlung](/de/guide/error-handling/strategies) - Fehlerbehandlungsstrategien
- [Testmethoden](/de/guide/testing/unit-tests) - RxJS-Testmethoden
- [RxJS-Antimuster](/de/guide/anti-patterns/) - Häufige Fehler und Lösungen
- [Pipeline](/de/guide/operators/pipeline) - Verkettung von Operatoren
