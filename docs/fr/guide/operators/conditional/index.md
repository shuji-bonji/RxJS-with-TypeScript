---
description: "Les opérateurs conditionnels de RxJS sont des opérateurs permettant de prendre des décisions conditionnelles sur les valeurs des flux, de définir des valeurs par défaut et d'évaluer des conditions. Avec defaultIfEmpty, every, isEmpty, etc., des scénarios pratiques comme le traitement des flux vides, la vérification de toutes les valeurs et la vérification d'existence peuvent être mis en œuvre avec la sécurité des types de TypeScript."
---

# Opérateurs conditionnels

Les opérateurs conditionnels de RxJS sont conçus pour effectuer **un jugement et une évaluation conditionnels** sur les valeurs d'un flux.
Par exemple, définir une valeur par défaut pour un flux vide ou vérifier que toutes les valeurs satisfont à une condition.
Ils peuvent être utilisés dans des scénarios pratiques.

Cette page présente chaque opérateur selon la structure suivante en trois étapes : « Syntaxe et fonctionnement de base », « Exemples d'utilisation typiques » et « Exemples de code pratique (avec interface utilisateur) ».

Comprendre les cas d'utilisation pour lesquels chaque opérateur est adapté et les combiner permet de concevoir des traitements réactifs plus robustes et conformes à l'intention.

> [!NOTE]
> `iif` et `defer` sont des **Fonctions de Création** (fonctions de création d'Observables), et non des opérateurs conditionnels. Voir [Chapitre 3 : Fonctions de Création](/fr/guide/creation-functions/) pour plus d'informations à ce sujet.

## Liste des opérateurs

Voici une liste des principaux opérateurs conditionnels et de leurs caractéristiques.

| Opérateur | Description |
|--------------|------|
| [defaultIfEmpty](./defaultIfEmpty.md) | Valeur alternative si aucune valeur n'est émise |
| [every](./every.md) | Évaluer si toutes les valeurs correspondent à la condition |
| [isEmpty](./isEmpty.md) | Vérifier si une valeur existe |

> Pour des **moyens pratiques de combiner des opérateurs** et des **applications basées sur des cas d'utilisation**, voir la section [Cas d'utilisation pratiques](./practical-use-cases.md).


## Pensez aussi à la connexion avec d'autres catégories

Les opérateurs conditionnels n'ont de valeur réelle que lorsqu'ils sont combinés avec d'autres opérateurs de transformation, de combinaison et utilitaires.
Par exemple, il est courant de les combiner avec `switchMap` et `catchError` pour effectuer un « changement d'API et traitement de récupération ».

Pour plus de cas d'utilisation pratiques, voir [Cas d'utilisation pratiques](./practical-use-cases.md).
