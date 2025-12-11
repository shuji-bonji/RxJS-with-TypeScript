---
description: RxJS multicasting-gerelateerde operators worden uitgelegd. Gebruik van share, shareReplay, publish en multicast, conversie van cold naar hot, efficiënte waarde distributie naar meerdere subscribers, maatregelen tegen geheugenlekkages, en praktische multicast strategieën worden geïntroduceerd. Realiseer type-veilig stream delen met TypeScript type-inferentie en leer implementatiepatronen die nuttig zijn voor performance optimalisatie.
---

# Operators Gebruikt voor Multicasting

In RxJS zijn er verschillende speciale operators beschikbaar om "multicasting" te realiseren, waarbij de output van dezelfde Observable wordt gedeeld met meerdere subscribers.

Deze pagina introduceert kort representatieve operators die verband houden met multicasting vanuit het **perspectief van operators**,
en organiseert hun gebruik en aandachtspunten.

> ❗ Voor conceptuele uitleg van multicasting en structurele beschrijving met Subject, en specifieke code voorbeelden,
> raadpleeg [Het Mechanisme van Multicasting](/nl/guide/subjects/multicasting).

## Belangrijkste Multicasting-gerelateerde Operators

| Operator | Kenmerken | Opmerkingen |
|----------|----------|-------------|
| **[share()](/nl/guide/operators/multicasting/share)** | Eenvoudigste multicast middel. Intern equivalent aan `publish().refCount()` | Voldoende voor de meeste use cases |
| **[shareReplay()](/nl/guide/operators/multicasting/shareReplay)** | Biedt recente waarden bij hernieuwde subscriptie naast multicasting | Wanneer hergebruik van status nodig is |
| `publish()` + `refCount()` | Multicast configuratie met beheersbare uitvoeringstiming | Klassieke en flexibele configuratie |
| `multicast()` | Low-level API die expliciet `Subject` doorgeeft | Effectief wanneer aangepaste Subject gewenst is |

## Vergelijking van Multicasting Patronen

| Operator | Kenmerken | Use Case |
|----------|----------|----------|
| **[share()](/nl/guide/operators/multicasting/share)** | Basis multicasting | Gelijktijdig gebruik door meerdere componenten |
| **[shareReplay(n)](/nl/guide/operators/multicasting/shareReplay)** | Buffert afgelopen n waarden | Vertraagde subscriptie/status delen |
| `publish() + refCount()` | Fijnere controle mogelijk | Wanneer geavanceerde controle nodig is |
| `multicast(() => new Subject())` | Volledige aanpassing | Wanneer speciaal Subject-type nodig is |

## Aandachtspunten bij Gebruik van Multicasting

1. **Timing begrijpen**: Begrijp dat ontvangen waarden verschillen afhankelijk van wanneer subscriptie start
2. **Levenscyclus beheer**: Vooral bij gebruik van `refCount`, voltooit de stream wanneer subscribers nul worden
3. **Foutafhandeling**: Wanneer een fout optreedt in een multicast Observable, beïnvloedt dit alle subscribers
4. **Geheugenbeheer**: Wees voorzichtig met geheugenlekkages bij gebruik van `shareReplay` e.d.
