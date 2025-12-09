---
description: Erklärung der Multicasting-bezogenen Operatoren in RxJS. Verwendungsunterschiede zwischen share, shareReplay, publish und multicast, Konvertierung von Cold zu Hot, effiziente Werteverteilung an mehrere Subscriber, Maßnahmen gegen Memory Leaks und mehr - praktische Multicast-Strategien werden vorgestellt. Mit TypeScript-Typinferenz wird typensichere Stream-Freigabe realisiert und Implementierungsmuster zur Performance-Optimierung vermittelt.
---

# Operatoren für Multicasting

In RxJS stehen mehrere spezielle Operatoren zur Verfügung, um "Multicasting" zu realisieren - das Teilen der Ausgabe eines Observables mit mehreren Subscribern.

Diese Seite stellt aus der **Perspektive der Operatoren** die wichtigsten Multicasting-bezogenen Operatoren vor und organisiert deren Verwendungsunterschiede und Vorsichtsmaßnahmen.

> ❗ Für konzeptionelle Erklärungen von Multicast, strukturelle Beschreibungen unter Verwendung von Subject und konkrete Codebeispiele siehe bitte
> [Multicasting-Mechanismus](/de/guide/subjects/multicasting).

## Hauptsächliche Multicasting-bezogene Operatoren

| Operator | Merkmale | Bemerkungen |
|--------------|------|------|
| **[share()](/de/guide/operators/multicasting/share)** | Einfachster Multicast-Mechanismus. Intern äquivalent zu `publish().refCount()` | Für die meisten Anwendungsfälle ausreichend |
| **[shareReplay()](/de/guide/operators/multicasting/shareReplay)** | Zusätzlich zu Multicast werden die letzten Werte bei erneutem Abonnement bereitgestellt | Wenn Zustandswiederverwendung erforderlich ist |
| `publish()` + `refCount()` | Multicast-Konfiguration mit kontrollierbarem Ausführungstiming | Klassische und flexible Konfiguration |
| `multicast()` | Low-Level-API, die explizit ein `Subject` übergibt | Nützlich bei Verwendung von benutzerdefinierten Subjects |

## Vergleich der Multicasting-Muster

| Operator | Merkmale | Anwendungsfall |
|------------|------|-------------|
| **[share()](/de/guide/operators/multicasting/share)** | Grundlegendes Multicasting | Gleichzeitige Nutzung in mehreren Komponenten |
| **[shareReplay(n)](/de/guide/operators/multicasting/shareReplay)** | Puffert die letzten n Werte | Verspätete Subscription/Zustandsfreigabe |
| `publish() + refCount()` | Feinere Kontrolle möglich | Wenn erweiterte Kontrolle erforderlich ist |
| `multicast(() => new Subject())` | Vollständige Anpassung | Wenn spezielle Subject-Typen benötigt werden |

## Vorsichtsmaßnahmen bei Verwendung von Multicasting

1. **Timing verstehen**: Verstehen, dass sich die empfangenen Werte je nach Zeitpunkt des Subscription-Starts unterscheiden
2. **Lifecycle-Management**: Besonders bei Verwendung von `refCount` wird der Stream beendet, wenn die Anzahl der Subscriber null wird
3. **Fehlerbehandlung**: Wenn ein Fehler im gemulticasteten Observable auftritt, betrifft dies alle Subscriber
4. **Speicherverwaltung**: Bei Verwendung von `shareReplay` etc. auf Memory Leaks achten
