---
description: RxJS voorwaardelijke operators voeren voorwaardelijke beoordelingen uit op waarden in streams en stellen standaardwaarden in of evalueren voorwaarden. Met defaultIfEmpty, every, isEmpty en andere operators kunt u praktische scenario's zoals het verwerken van lege streams, volledige controles en bestaanscontroles implementeren met TypeScript type-veiligheid.
---

# Voorwaardelijke Operators

RxJS voorwaardelijke operators zijn bedoeld om **voorwaardelijke beoordelingen en evaluaties** uit te voeren op stream-waarden.
Ze kunnen worden gebruikt in praktische scenario's zoals het instellen van standaardwaarden voor lege streams of het verifiëren dat alle waarden aan een voorwaarde voldoen.

Deze pagina introduceert elke operator in drie fasen: "Basissyntax en Werking", "Typische Gebruiksvoorbeelden" en "Praktische Code Voorbeelden (met UI)".

Door te begrijpen welke use cases geschikt zijn voor elke operator en deze te combineren,
kunt u een robuustere en meer intentionele reactieve verwerkingsontwerp creëren.

> [!NOTE]
> `iif` en `defer` zijn **Creation Functions** (Observable aanmaakfuncties), geen voorwaardelijke operators. Raadpleeg [Hoofdstuk 3: Creation Functions](/nl/guide/creation-functions/) voor deze functies.

## Operator Overzicht

Hieronder volgt een overzicht van de belangrijkste voorwaardelijke operators en hun kenmerken.

| Operator | Beschrijving |
|----------|-------------|
| [defaultIfEmpty](./defaultIfEmpty.md) | Alternatieve waarde wanneer geen waarden worden uitgezonden |
| [every](./every.md) | Evalueert of alle waarden aan een voorwaarde voldoen |
| [isEmpty](./isEmpty.md) | Controleert of er uitgezonden waarden bestaan |

> Voor **praktische combinaties van operators** en **use case-gebaseerde toepassingsvoorbeelden** verwijzen we naar het [Praktische Use Cases](./practical-use-cases.md) gedeelte verderop.


## Denk ook aan Integratie met Andere Categorieën

Voorwaardelijke operators tonen hun ware waarde wanneer ze worden gecombineerd met andere transformatie-, combinatie- en utility operators.
Een veelvoorkomend patroon is bijvoorbeeld het combineren met `switchMap` of `catchError` om "API-switching en recovery-verwerking" uit te voeren.

Voor meer praktische gebruiksvoorbeelden verwijzen we naar [Praktische Use Cases](./practical-use-cases.md) voor gedetailleerde uitleg.
