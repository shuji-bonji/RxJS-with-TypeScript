---
description: "Explique les opérateurs de multidiffusion de RxJS, y compris les stratégies pratiques de multidiffusion telles que l'utilisation de share, shareReplay, publish et multicast, la conversion cold vers hot, la livraison efficace de valeurs à des abonnés multiples et la prévention des fuites de mémoire. Apprenez l'inférence de type TypeScript pour un partage de flux type-safe et des modèles d'implémentation pour l'optimisation des performances."
---

# Opérateurs utilisés dans la multidiffusion

RxJS fournit plusieurs opérateurs dédiés à la réalisation de la « multidiffusion », où la même sortie Observable est partagée pour plusieurs abonnés.

Cette page présente brièvement les opérateurs typiques pertinents pour la multidiffusion **du point de vue des opérateurs**,
et organise leur utilisation et les points à noter.

> ❗ Pour une explication structurelle du concept de multidiffusion et de l'utilisation de Subject, ainsi que des exemples de code concrets, voir
> [Comment fonctionne la multidiffusion](/fr/guide/subjects/multicasting).

## Principaux opérateurs liés à la multidiffusion

| Opérateur | Caractéristiques | Remarques |
|--------------|------|------|
| **[share()](/fr/guide/operators/multicasting/share)** | Le moyen le plus simple de multidiffusion. Équivalent en interne à `publish().refCount()` | Suffisant pour de nombreux cas d'utilisation |
| **[shareReplay()](/fr/guide/operators/multicasting/shareReplay)** | En plus de la multidiffusion, fournit la valeur la plus récente lors du réabonnement | Quand la réutilisation de l'état est nécessaire |
| `publish()` + `refCount()` | Configuration de multidiffusion avec timing d'exécution contrôlable | Configuration classique et flexible |
| `multicast()` | API de bas niveau pour passer explicitement `Subject` | Utile si vous voulez utiliser des Subjects personnalisés |

## Comparaison des modèles de multidiffusion

| Opérateur | Caractéristiques | Cas d'utilisation |
|------------|------|-------------|
| **[share()](/fr/guide/operators/multicasting/share)** | Multidiffusion de base | Utilisation simultanée dans plusieurs composants |
| **[shareReplay(n)](/fr/guide/operators/multicasting/shareReplay)** | Met en tampon les n dernières valeurs | Abonnement différé/partage d'état |
| `publish() + refCount()` | Contrôle plus fin possible | Quand un contrôle avancé est nécessaire |
| `multicast(() => new Subject())` | Personnalisation complète | Quand un type de Subject spécial est nécessaire |

## Notes sur l'utilisation de la multidiffusion

1. **Compréhension du timing** : Comprenez que la valeur que vous recevez dépend du moment où l'abonnement commence
2. **Gestion du cycle de vie** : En particulier lors de l'utilisation de `refCount`, le flux se termine quand le nombre d'abonnés atteint zéro
3. **Gestion des erreurs** : Une erreur dans un Observable multidiffusé affectera tous les abonnés
4. **Gestion de la mémoire** : Attention aux fuites de mémoire lors de l'utilisation de `shareReplay`, etc.
