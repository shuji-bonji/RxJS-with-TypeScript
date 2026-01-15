---
description: shareReplay ist ein RxJS-Multicast-Operator, der zusätzlich zu Multicasting vergangene Werte puffert und auch verspäteten Subscribern bereitstellt. Optimal für API-Response-Caching, Konfigurationsfreigabe, State Management und andere Szenarien, in denen vergangene Werte gespeichert und an mehrere Subscriber verteilt werden sollen. Memory Leak-Prävention ist mit refCount- und windowTime-Optionen möglich, TypeScript-Typinferenz ermöglicht typensichere Cache-Verarbeitung.
---

# shareReplay - Cache und Teilen

Der `shareReplay()`-Operator realisiert wie `share()` Multicasting, merkt sich aber zusätzlich **eine bestimmte Anzahl vergangener Werte** und stellt sie auch später beitretenden Subscribern bereit.

Dies ermöglicht fortgeschrittenere Anwendungsfälle wie API-Response-Caching und Zustandsfreigabe.

[📘 RxJS Offizielle Dokumentation - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 Grundlegende Verwendung

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// shareReplay verwenden (Puffergröße 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Quelle: ${value}`)),
  shareReplay(2) // Die letzten 2 Werte puffern
);

// Erster Subscriber
console.log('Observer 1 Subscription gestartet');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3.5 Sekunden später zweiten Subscriber hinzufügen
setTimeout(() => {
  console.log('Observer 2 Subscription gestartet - erhält die letzten 2 Werte');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### Ausführungsergebnis

```
Observer 1 Subscription gestartet
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Quelle: 2
Observer 1: 2
Quelle: 3
Observer 1: 3
Observer 2 Subscription gestartet - erhält die letzten 2 Werte
Observer 2: 2  // ← Gepufferter vergangener Wert
Observer 2: 3  // ← Gepufferter vergangener Wert
Quelle: 4
Observer 1: 4
Observer 2: 4
```

**Wichtige Punkte**:
- Verspätete Subscriber können gepufferte vergangene Werte sofort erhalten
- So viele Werte wie die Puffergröße werden gespeichert (in diesem Beispiel 2)

## 💡 Syntax von shareReplay()

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### Parameter

| Parameter | Typ | Beschreibung | Standard |
|-----------|---|------|----------|
| `bufferSize` | `number` | Anzahl der zu puffernden Werte | `Infinity` |
| `windowTime` | `number` | Gültigkeitsdauer des Puffers (Millisekunden) | `Infinity` |
| `scheduler` | `SchedulerLike` | Scheduler für Timing-Kontrolle | - |

### Konfigurationsobjekt (RxJS 7+)

```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // Ob bei null Subscribern abgemeldet werden soll
  scheduler?: SchedulerLike;
}
```

## 📊 Unterschied zwischen share und shareReplay

### Verhalten von share()

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Quelle: ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 Subscription gestartet');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Ausführungsergebnis**:
```
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Observer 2 Subscription gestartet
Quelle: 2
Observer 1: 2
Observer 2: 2  // ← Kann vergangene Werte (0, 1) nicht erhalten
```

### Verhalten von shareReplay()

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Quelle: ${value}`)),
  shareReplay(2) // Die letzten 2 Werte puffern
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 Subscription gestartet');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Ausführungsergebnis**:
```
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Observer 2 Subscription gestartet
Observer 2: 0  // ← Gepufferter vergangener Wert
Observer 2: 1  // ← Gepufferter vergangener Wert
Quelle: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Praktische Anwendungsfälle

### 1. Caching von API-Responses

```typescript
import { Observable } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, shareReplay, tap } from 'rxjs';

interface User {
  id: number;
  name: string;
  username: string;
  email: string;
}

class UserService {
  // Benutzerinformationen cachen
  private userCache$ = ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
    tap(() => console.log('API-Request ausgeführt')),
    shareReplay(1) // Den letzten Wert dauerhaft cachen
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// Erste Komponente
userService.getUser().subscribe(user => {
  console.log('Komponente 1:', user);
});

// 2 Sekunden später andere Komponente
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('Komponente 2:', user); // ← Aus Cache abrufen, kein API-Request
  });
}, 2000);
```

**Ausführungsergebnis**:
```
API-Request ausgeführt
Komponente 1: { id: 1, name: "John" }
Komponente 2: { id: 1, name: "John" }  // ← Kein API-Request
```

### 2. Freigabe von Konfigurationsinformationen

```typescript
import { of } from 'rxjs';
import { delay, shareReplay, tap } from 'rxjs';

// Anwendungskonfiguration abrufen (nur beim ersten Mal ausgeführt)
const appConfig$ = of({
  apiUrl: 'https://api.example.com',
  theme: 'dark',
  language: 'de'
}).pipe(
  delay(1000), // Laden simulieren
  tap(() => console.log('Konfiguration geladen')),
  shareReplay(1)
);

// Konfiguration in mehreren Services verwenden
appConfig$.subscribe(config => console.log('Service A:', config.apiUrl));
appConfig$.subscribe(config => console.log('Service B:', config.theme));
appConfig$.subscribe(config => console.log('Service C:', config.language));
```

**Ausführungsergebnis**:
```
Konfiguration geladen
Service A: https://api.example.com
Service B: dark
Service C: de
```

### 3. Cache mit Zeitbeschränkung

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, tap } from 'rxjs';

// Nur 5 Sekunden cachen (TODO-Daten als Beispiel)
const todoData$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1').pipe(
  tap(() => console.log('TODO-Daten abgerufen')),
  shareReplay({
    bufferSize: 1,
    windowTime: 5000, // 5 Sekunden gültig
    refCount: true    // Bei null Subscribern abmelden
  })
);

// Erste Subscription
todoData$.subscribe(data => console.log('Abruf 1:', data));

// Nach 3 Sekunden (Cache gültig)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Abruf 2:', data)); // Aus Cache
}, 3000);

// Nach 6 Sekunden (Cache abgelaufen)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Abruf 3:', data)); // Neuer Request
}, 6000);
```

## ⚠️ Achtung bei Memory Leaks

`shareReplay()` hält Werte dauerhaft im Puffer, was bei unsachgemäßer Verwaltung zu Memory Leaks führen kann.

### Problematischer Code

```typescript
// ❌ Gefahr eines Memory Leaks
const infiniteStream$ = interval(1000).pipe(
  shareReplay() // bufferSize nicht angegeben = Infinity
);

// Dieser Stream sammelt unendlich viele Werte an
```

### Empfohlene Gegenmaßnahmen

```typescript
// ✅ Puffergröße begrenzen
const safeStream$ = interval(1000).pipe(
  shareReplay(1) // Nur den letzten Wert behalten
);

// ✅ refCount verwenden
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    refCount: true // Puffer löschen bei null Subscribern
  })
);

// ✅ Zeitbeschränkung festlegen
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    windowTime: 10000 // Nach 10 Sekunden ablaufen
  })
);
```

## 🎯 Auswahl der Puffergröße

| Puffergröße | Anwendungsfall | Beispiel |
|--------------|-----------|---|
| `1` | Nur der aktuelle Zustand erforderlich | Aktuelle Benutzerinformationen, Einstellungen |
| `3-5` | Historie der letzten Einträge erforderlich | Chat-Verlauf, Benachrichtigungsverlauf |
| `Infinity` | Gesamte Historie erforderlich | Logs, Audit-Trail (Vorsicht geboten) |

## 🔄 Verwandte Operatoren

- **[share()](/de/guide/operators/multicasting/share)** - Einfaches Multicast (ohne Puffer)
- **[publish()](/de/guide/subjects/multicasting)** - Low-Level-Multicast-Kontrolle
- **[ReplaySubject](/de/guide/subjects/types-of-subject)** - Grundlage von shareReplay

## Zusammenfassung

Der `shareReplay()`-Operator bietet:
- Pufferung vergangener Werte und Bereitstellung auch für verspätete Subscriber
- Optimal für API-Response-Caching
- Vorsicht bei Memory Leaks erforderlich
- Sichere Verwendung mit `refCount` und `windowTime` möglich

Wenn Zustandsfreigabe oder Caching erforderlich ist, ist `shareReplay()` ein sehr mächtiges Werkzeug, aber es ist wichtig, angemessene Puffergrößen und Ablaufeinstellungen vorzunehmen.

## 🔗 Verwandte Abschnitte

- **[Häufige Fehler und Lösungen](/de/guide/anti-patterns/)** - Korrekte Verwendung von shareReplay und Memory Leak-Gegenmaßnahmen
- **[share()](/de/guide/operators/multicasting/share)** - Einfaches Multicast
- **[ReplaySubject](/de/guide/subjects/types-of-subject)** - Grundlage von shareReplay
