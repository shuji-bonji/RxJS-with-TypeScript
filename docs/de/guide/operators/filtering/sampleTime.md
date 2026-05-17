---
description: Der sampleTime-Operator ist ein RxJS-Filteroperator, der in angegebenen Zeitintervallen regelmäßig den neuesten Wert des Streams sampelt. Ideal für regelmäßige Snapshot-Erfassung.
---

# sampleTime - Regelmäßig Sampeln

Der `sampleTime`-Operator **sampelt regelmäßig in angegebenen Zeitintervallen** den **neuesten Wert** des Quell-Observable und gibt ihn aus.
Wie ein regelmäßiger Snapshot wird der neueste Wert zu diesem Zeitpunkt erfasst.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Sample alle 2 Sekunden');
});
```

**Ablauf**:
1. Timer wird alle 2 Sekunden regelmäßig ausgelöst
2. Wenn zu diesem Zeitpunkt neuestes Click-Ereignis vorhanden, ausgeben
3. Wenn keine Werte während Sample-Zeitraum, nichts ausgeben

[🌐 RxJS Offizielle Dokumentation - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

> [!WARNING] Hinweis für Produktionscode
> Das obige Beispiel lässt die Abmeldung von `fromEvent` zur Vereinfachung der Erklärung weg. Verwenden Sie in echtem Code `takeUntil(destroy$)`, `take(N)` oder `Subscription.unsubscribe()`, um den Lebenszyklus explizit zu verwalten. Details: [Schwierigkeiten überwinden: Lebenszyklus-Verwaltung](/de/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Typische Anwendungsmuster

- **Regelmäßige Sensordatenerfassung**: Neueste Temperatur oder Positionsinformationen jede Sekunde
- **Echtzeit-Dashboard**: Regelmäßige Zustandsaktualisierung
- **Leistungsüberwachung**: Metriken-Erfassung in festen Intervallen
- **Spiel-Frame-Verarbeitung**: FPS-Steuerung durch regelmäßiges Sampling

[Vollständige Codebeispiele mit deutscher Übersetzung]

## 🆚 Vergleich mit ähnlichen Operatoren

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: Neuesten Wert zu diesem Zeitpunkt alle 1 Sekunde sampeln
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Beispielausgabe: 2, 5, 8 (Snapshot alle 1 Sekunde)

// throttleTime: Ersten Wert ausgeben, dann 1 Sekunde ignorieren
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Beispielausgabe: 0, 3, 6, 9 (erster Wert jedes Zeitraums)

// auditTime: Letzten Wert des Zeitraums 1 Sekunde nach erstem Wert ausgeben
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Beispielausgabe: 2, 5, 8 (letzter Wert jedes Zeitraums)
```

| Operator | Auslösezeitpunkt | Ausgegebener Wert | Anwendungsfall |
|:---|:---|:---|:---|
| `sampleTime(1000)` | **Regelmäßiges Timing alle 1 Sekunde** | Neuester Wert zu diesem Zeitpunkt | Regelmäßiger Snapshot |
| `throttleTime(1000)` | 1 Sekunde nach Wertempfang ignorieren | Erster Wert zu Zeitraumbeginn | Ereignisreduzierung |
| `auditTime(1000)` | 1 Sekunde nach Wertempfang | Letzter Wert im Zeitraum | Neuester Zustand im Zeitraum |

## 📚 Verwandte Operatoren

- **[sample](https://rxjs.dev/api/operators/sample)** - Sampling mit anderem Observable als Trigger (Offizielle Dokumentation)
- **[throttleTime](./throttleTime)** - Ersten Wert zu Zeitraumbeginn abrufen
- **[auditTime](./auditTime)** - Letzten Wert bei Zeitraumende abrufen
- **[debounceTime](./debounceTime)** - Wert nach Ruhe ausgeben

## Zusammenfassung

Der `sampleTime`-Operator sampelt regelmäßig in angegebenen Zeitintervallen den neuesten Wert.

- ✅ Ideal für regelmäßige Snapshot-Erfassung
- ✅ Effektiv zur Reduzierung hochfrequenter Streams
- ✅ Gute Speichereffizienz (nur neuester Wert gespeichert)
- ✅ Ideal für Dashboard und Überwachung
- ⚠️ Nichts ausgegeben wenn keine Werte während Sample-Zeitraum
- ⚠️ Wartezeit bis erstes Sample
- ⚠️ Abschluss wird beim nächsten Sample-Timing weitergegeben
